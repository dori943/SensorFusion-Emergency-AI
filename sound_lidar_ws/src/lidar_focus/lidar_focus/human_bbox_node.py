import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree


def read_points(msg):
    """PointCloud2 → numpy 2D array"""
    pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)


class HumanBBoxNode(Node):
    """
    파이프라인:
    /filtered_cloud (전처리 완료된 포인트클라우드)
        ↓
    DBSCAN 클러스터링 (segmentation)
        ↓
    사람 크기 필터 (bbox 기준)
        ↓
    /human_cloud    → 사람 포인트클라우드 (RViz2 PointCloud2)
    /human_bbox     → 3D BBox 마커 (RViz2 MarkerArray)
    """

    def __init__(self):
        super().__init__('human_bbox_node')

        # ── 클러스터링 파라미터 ──────────────────────────
        self.declare_parameter('cluster_eps', 0.3)        # 클러스터 반경 (m)
        self.declare_parameter('cluster_min_points', 5)   # 최소 포인트 수

        # ── 사람 크기 필터 파라미터 ─────────────────────
        self.declare_parameter('person_height_min', 0.5)  # 최소 높이 (m)
        self.declare_parameter('person_height_max', 2.2)  # 최대 높이 (m)
        self.declare_parameter('person_width_min', 0.2)   # 최소 너비 (m)
        self.declare_parameter('person_width_max', 1.2)   # 최대 너비 (m)
        self.declare_parameter('person_points_min', 5)    # 최소 포인트 수

        # ── ROS I/O ──────────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2, '/filtered_cloud', self.callback, 10)
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/human_cloud', 10)
        self.pub_bbox = self.create_publisher(
            MarkerArray, '/human_bbox', 10)

        self.get_logger().info('HumanBBoxNode started.')
        self.get_logger().info('Subscribing to /filtered_cloud')
        self.get_logger().info('Publishing to /human_cloud and /human_bbox')

    # ──────────────────────────────────────────────────────
    # DBSCAN 클러스터링 (segmentation)
    # ──────────────────────────────────────────────────────
    def dbscan(self, points, eps, min_points):
        """순수 numpy + scipy로 DBSCAN 구현"""
        n = len(points)
        labels = -np.ones(n, dtype=int)  # -1 = 노이즈
        cluster_id = 0
        visited = np.zeros(n, dtype=bool)
        tree = KDTree(points)

        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            neighbors = tree.query_ball_point(points[i], eps)

            if len(neighbors) < min_points:
                labels[i] = -1  # 노이즈
                continue

            labels[i] = cluster_id
            seed_set = list(neighbors)

            j = 0
            while j < len(seed_set):
                q = seed_set[j]
                if not visited[q]:
                    visited[q] = True
                    new_neighbors = tree.query_ball_point(points[q], eps)
                    if len(new_neighbors) >= min_points:
                        seed_set += [x for x in new_neighbors if x not in seed_set]
                if labels[q] == -1:
                    labels[q] = cluster_id
                j += 1

            cluster_id += 1

        return labels

    # ──────────────────────────────────────────────────────
    # 사람 크기 필터
    # ──────────────────────────────────────────────────────
    def is_human(self, cluster):
        if len(cluster) < self.get_parameter('person_points_min').value:
            return False

        min_pt = cluster.min(axis=0)
        max_pt = cluster.max(axis=0)
        dx = max_pt[0] - min_pt[0]  # 너비
        dy = max_pt[1] - min_pt[1]  # 깊이
        dz = max_pt[2] - min_pt[2]  # 높이

        h_min = self.get_parameter('person_height_min').value
        h_max = self.get_parameter('person_height_max').value
        w_min = self.get_parameter('person_width_min').value
        w_max = self.get_parameter('person_width_max').value

        return (
            h_min < dz < h_max and
            w_min < dx < w_max and
            w_min < dy < w_max
        )

    # ──────────────────────────────────────────────────────
    # BBox 마커 생성
    # ──────────────────────────────────────────────────────
    def make_bbox_marker(self, cluster, marker_id, header):
        min_pt = cluster.min(axis=0)
        max_pt = cluster.max(axis=0)
        center = (min_pt + max_pt) / 2.0
        dx = max_pt[0] - min_pt[0]
        dy = max_pt[1] - min_pt[1]
        dz = max_pt[2] - min_pt[2]

        # 낙상 감지: bbox 높이/너비 비율
        ratio = dz / max(dx, dy, 0.01)
        if ratio < 0.8:
            # 낙상 의심 → 빨간색
            color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            self.get_logger().warn(f'[FALL DETECTED] Person {marker_id} ratio={ratio:.2f}')
        else:
            # 정상 → 초록색
            color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.4)

        # BBox (외곽선)
        bbox = Marker()
        bbox.header = header
        bbox.ns = 'human_bbox'
        bbox.id = marker_id
        bbox.type = Marker.CUBE
        bbox.action = Marker.ADD
        bbox.pose.position.x = float(center[0])
        bbox.pose.position.y = float(center[1])
        bbox.pose.position.z = float(center[2])
        bbox.pose.orientation.w = 1.0
        bbox.scale.x = float(max(dx, 0.01))
        bbox.scale.y = float(max(dy, 0.01))
        bbox.scale.z = float(max(dz, 0.01))
        bbox.color = color
        bbox.lifetime.sec = 1

        # 텍스트 (ID + 크기 정보)
        text = Marker()
        text.header = header
        text.ns = 'human_text'
        text.id = marker_id + 100
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(center[0])
        text.pose.position.y = float(center[1])
        text.pose.position.z = float(max_pt[2]) + 0.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f'P{marker_id} H:{dz:.1f}m W:{dx:.1f}m'
        text.lifetime.sec = 1

        return bbox, text

    # ──────────────────────────────────────────────────────
    # 메인 콜백
    # ──────────────────────────────────────────────────────
    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        # DBSCAN 클러스터링
        eps = self.get_parameter('cluster_eps').value
        min_pts = self.get_parameter('cluster_min_points').value
        labels = self.dbscan(points, eps, min_pts)

        # 클러스터별 분리
        clusters = []
        for label in set(labels):
            if label < 0:
                continue
            clusters.append(points[labels == label])

        if not clusters:
            return

        # 사람 크기 필터
        human_clusters = [c for c in clusters if self.is_human(c)]

        if not human_clusters:
            return

        self.get_logger().info(f'Detected {len(human_clusters)} person(s)')

        # 사람 포인트클라우드 퍼블리시
        all_points = np.vstack(human_clusters)
        self.pub_cloud.publish(pc2.create_cloud_xyz32(msg.header, all_points))

        # BBox 마커 퍼블리시
        marker_array = MarkerArray()

        # 기존 마커 클리어
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, cluster in enumerate(human_clusters):
            bbox, text = self.make_bbox_marker(cluster, i, msg.header)
            marker_array.markers.append(bbox)
            marker_array.markers.append(text)

        self.pub_bbox.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HumanBBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
