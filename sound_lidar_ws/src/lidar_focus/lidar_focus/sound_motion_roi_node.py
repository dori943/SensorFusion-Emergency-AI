import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
from sound_interfaces.msg import SoundEvent
from sklearn.cluster import DBSCAN


def read_xyz(msg):
    points = pc2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    if points is None or len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return points.astype(np.float32, copy=False)


def make_cloud(header, points):
    if len(points) == 0:
        points = np.zeros((0, 3), dtype=np.float32)
    return pc2.create_cloud_xyz32(header, points.astype(np.float32, copy=False))


def angle_delta_deg(values, target):
    return (values - target + 180.0) % 360.0 - 180.0


class SoundMotionRoiNode(Node):
    """Sound-gated ROI extraction followed by lightweight background subtraction."""

    def __init__(self):
        super().__init__("sound_motion_roi_node")

        self.declare_parameter("lidar_topic", "/unilidar/cloud")
        self.declare_parameter("sound_topic", "/sound_events")
        self.declare_parameter("roi_cloud_topic", "/fusion/sound_roi_cloud")
        self.declare_parameter("moving_cloud_topic", "/fusion/moving_roi_cloud")
        self.declare_parameter("sector_width_deg", 45.0)
        self.declare_parameter("sound_timeout_sec", 1.5)
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("z_min", -0.3)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("range_min", 0.5)
        self.declare_parameter("range_max", 15.0)
        self.declare_parameter("background_frames", 80)
        self.declare_parameter("background_voxel_size", 0.12)
        self.declare_parameter("roi_voxel_size", 0.06)
        self.declare_parameter("neighbor_margin_voxels", 1)
        self.declare_parameter("publish_empty_cloud", True)

        # Stage 2: DBSCAN clustering parameters
        self.declare_parameter("high_res_voxel_size", 0.05)
        self.declare_parameter("cluster_eps", 0.25)
        self.declare_parameter("cluster_min_points", 8)

        self._target_angle = None
        self._last_sound_time = self.get_clock().now()
        self._background_ready = False
        self._background_frames = []
        self._background_keys = set()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            SoundEvent,
            self.get_parameter("sound_topic").value,
            self._sound_cb,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self._cloud_cb,
            sensor_qos,
        )

        self._roi_pub = self.create_publisher(
            PointCloud2, self.get_parameter("roi_cloud_topic").value, sensor_qos
        )
        self._moving_pub = self.create_publisher(
            PointCloud2, self.get_parameter("moving_cloud_topic").value, sensor_qos
        )
        self._cluster_cloud_pub = self.create_publisher(
            PointCloud2, "/fusion/cluster_cloud", sensor_qos
        )
        self._cluster_markers_pub = self.create_publisher(
            MarkerArray, "/fusion/cluster_markers", sensor_qos
        )

        self.create_timer(2.0, self._status_log)
        self.get_logger().info(
            "SoundMotionRoiNode started: sound ROI + voxel background subtraction"
        )

    def _sound_cb(self, msg):
        if not msg.is_active:
            return
        if msg.confidence < self.get_parameter("min_confidence").value:
            return

        self._target_angle = float(msg.angle)
        self._last_sound_time = self.get_clock().now()
        self.get_logger().info(
            f"Sound ROI angle updated: {self._target_angle:.1f} deg "
            f"(conf={msg.confidence:.2f}, amp={msg.amplitude:.1f})"
        )

    def _cloud_cb(self, msg):
        points = read_xyz(msg)
        if len(points) == 0:
            self._publish_empty_if_enabled(msg.header)
            return

        points = self._basic_filter(points)
        if len(points) == 0:
            self._publish_empty_if_enabled(msg.header)
            return

        if not self._background_ready:
            self._collect_background(points)
            self._publish_empty_if_enabled(msg.header)
            return

        if not self._has_recent_sound():
            self._publish_empty_if_enabled(msg.header)
            return

        roi = self._angle_roi(points)
        if len(roi) == 0:
            self._publish_empty_if_enabled(msg.header)
            return

        roi = self._voxel_downsample(
            roi, self.get_parameter("roi_voxel_size").value
        )
        self._roi_pub.publish(make_cloud(msg.header, roi))

        moving = self._background_subtract(roi)
        if len(moving) == 0:
            self._publish_empty_moving_if_enabled(msg.header)
            return

        self._moving_pub.publish(make_cloud(msg.header, moving))

        # Stage 2: voxel downsample + DBSCAN clustering
        clusters = self._cluster_moving_points(moving)
        if clusters:
            self._publish_clusters(clusters, msg.header)

    def _cluster_moving_points(self, points):
        min_pts = self.get_parameter("cluster_min_points").value
        if len(points) < min_pts:
            return []

        fine = self._voxel_downsample(
            points, self.get_parameter("high_res_voxel_size").value
        )
        if len(fine) < min_pts:
            return []

        labels = DBSCAN(
            eps=self.get_parameter("cluster_eps").value,
            min_samples=min_pts,
        ).fit_predict(fine)

        clusters = []
        for label in np.unique(labels):
            if label < 0:
                continue
            clusters.append(fine[labels == label])

        self.get_logger().debug(f"Stage2: {len(clusters)} clusters from {len(fine)} pts")
        return clusters

    def _publish_clusters(self, clusters, header):
        # merge all cluster points into one cloud for visualization
        all_pts = np.vstack(clusters)
        self._cluster_cloud_pub.publish(make_cloud(header, all_pts))

        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, cluster in enumerate(clusters):
            min_pt = cluster.min(axis=0)
            max_pt = cluster.max(axis=0)
            center = (min_pt + max_pt) / 2.0
            size = max_pt - min_pt

            m = Marker()
            m.header = header
            m.ns = "cluster_bbox"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            m.pose.orientation.w = 1.0
            m.scale.x = float(max(size[0], 0.05))
            m.scale.y = float(max(size[1], 0.05))
            m.scale.z = float(max(size[2], 0.05))
            m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.35)
            m.lifetime.sec = 1
            markers.markers.append(m)

            t = Marker()
            t.header = header
            t.ns = "cluster_text"
            t.id = i + 100
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(center[0])
            t.pose.position.y = float(center[1])
            t.pose.position.z = float(max_pt[2]) + 0.15
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"C{i} n={len(cluster)} h={size[2]:.2f}m"
            t.lifetime.sec = 1
            markers.markers.append(t)

        self._cluster_markers_pub.publish(markers)
        self.get_logger().info(
            f"Stage2: published {len(clusters)} clusters"
        )

    def _basic_filter(self, points):
        ranges = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            (ranges >= self.get_parameter("range_min").value)
            & (ranges <= self.get_parameter("range_max").value)
            & (points[:, 2] >= self.get_parameter("z_min").value)
            & (points[:, 2] <= self.get_parameter("z_max").value)
        )
        return points[mask]

    def _angle_roi(self, points):
        point_angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        target = self._target_angle
        half_width = self.get_parameter("sector_width_deg").value
        mask = np.abs(angle_delta_deg(point_angles, target)) <= half_width
        return points[mask]

    def _collect_background(self, points):
        voxel_size = self.get_parameter("background_voxel_size").value
        self._background_frames.append(self._voxel_downsample(points, voxel_size))

        frame_count = self.get_parameter("background_frames").value
        self.get_logger().info(
            f"Collecting background: {len(self._background_frames)}/{frame_count}"
        )

        if len(self._background_frames) < frame_count:
            return

        background = np.vstack(self._background_frames)
        background = self._voxel_downsample(background, voxel_size)
        self._background_keys = self._points_to_key_set(background, voxel_size)
        self._background_frames.clear()
        self._background_ready = True
        self.get_logger().info(
            f"Background ready: {len(self._background_keys)} occupied voxels"
        )

    def _background_subtract(self, points):
        voxel_size = self.get_parameter("background_voxel_size").value
        margin = int(self.get_parameter("neighbor_margin_voxels").value)
        keys = np.floor(points / voxel_size).astype(np.int32)

        dynamic_mask = np.ones(len(points), dtype=bool)
        for i, key in enumerate(keys):
            if self._key_near_background(key, margin):
                dynamic_mask[i] = False

        return points[dynamic_mask]

    def _key_near_background(self, key, margin):
        if margin <= 0:
            return tuple(key.tolist()) in self._background_keys

        x, y, z = key.tolist()
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                for dz in range(-margin, margin + 1):
                    if (x + dx, y + dy, z + dz) in self._background_keys:
                        return True
        return False

    def _points_to_key_set(self, points, voxel_size):
        keys = np.floor(points / voxel_size).astype(np.int32)
        return {tuple(k.tolist()) for k in keys}

    def _voxel_downsample(self, points, voxel_size):
        if len(points) == 0:
            return points
        keys = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(unique_idx)]

    def _has_recent_sound(self):
        if self._target_angle is None:
            return False
        elapsed = (self.get_clock().now() - self._last_sound_time).nanoseconds / 1e9
        return elapsed <= self.get_parameter("sound_timeout_sec").value

    def _publish_empty_if_enabled(self, header):
        if self.get_parameter("publish_empty_cloud").value:
            empty = make_cloud(header, np.zeros((0, 3), dtype=np.float32))
            self._roi_pub.publish(empty)
            self._moving_pub.publish(empty)

    def _publish_empty_moving_if_enabled(self, header):
        if self.get_parameter("publish_empty_cloud").value:
            empty = make_cloud(header, np.zeros((0, 3), dtype=np.float32))
            self._moving_pub.publish(empty)

    def _status_log(self):
        if not self._background_ready:
            return
        if not self._has_recent_sound():
            self.get_logger().info("Waiting for recent sound trigger")


def main(args=None):
    rclpy.init(args=args)
    node = SoundMotionRoiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
