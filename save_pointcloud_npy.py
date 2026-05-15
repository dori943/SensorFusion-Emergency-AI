import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class Saver(Node):
    def __init__(self):
        super().__init__('saver')
        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.callback, 10)
        self.saved = False

    def callback(self, msg):
        if self.saved:
            return
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        points = np.array([[p[0],p[1],p[2]] for p in pts], dtype=np.float32)
        np.save('pointcloud.npy', points)
        print(f'Saved {len(points)} points to pointcloud.npy')
        self.saved = True

rclpy.init()
node = Saver()
rclpy.spin(node)