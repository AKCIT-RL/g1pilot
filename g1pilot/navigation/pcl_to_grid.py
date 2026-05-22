#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class PclToGridNode(Node):
    def __init__(self):
        super().__init__('pcl_to_grid')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('width_m', 40.0)
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('min_z', 0.1)
        self.declare_parameter('max_z', 1.0)
        
        self.res = self.get_parameter('resolution').value
        self.w_m = self.get_parameter('width_m').value
        self.h_m = self.get_parameter('height_m').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        
        self.width = int(self.w_m / self.res)
        self.height = int(self.h_m / self.res)
        self.origin_x = -self.w_m / 2.0
        self.origin_y = -self.h_m / 2.0
        
        self.sub = self.create_subscription(PointCloud2, '/lidar_odometry/localmap_points', self.callback, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.get_logger().info(f"PCL to Grid initialized: {self.width}x{self.height} @ {self.res}m/px")

    def callback(self, msg):
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            # Converting to a standard float numpy array (N, 3)
            # This avoids the VoidDType error with structured arrays
            pts_list = [[p[0], p[1], p[2]] for p in points]
            if not pts_list:
                return
            
            pts = np.array(pts_list, dtype=np.float32)
                
            # Filtering by Z (height)
            mask = (pts[:, 2] > self.min_z) & (pts[:, 2] < self.max_z)
            pts = pts[mask]
            
            if pts.size == 0:
                return
            
            # Creating the Grid map (OccupancyGrid)
            grid = np.zeros((self.height, self.width), dtype=np.int8)
            
            # Converting world to grid coordinates
            ix = ((pts[:, 0] - self.origin_x) / self.res).astype(int)
            iy = ((pts[:, 1] - self.origin_y) / self.res).astype(int)
            
            # Filtering valid bounds
            valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
            ix = ix[valid]
            iy = iy[valid]
            
            # Setting obstacles (100 = occupied)
            grid[iy, ix] = 100
            
            # Building and Publishing ROS message
            occ = OccupancyGrid()
            occ.header = msg.header
            occ.header.frame_id = 'map'
            occ.info.resolution = self.res
            occ.info.width = self.width
            occ.info.height = self.height
            occ.info.origin.position.x = self.origin_x
            occ.info.origin.position.y = self.origin_y
            occ.info.origin.orientation.w = 1.0
            occ.data = grid.flatten().tolist()
            
            self.pub.publish(occ)
        except Exception as e:
            self.get_logger().error(f"Erro no processamento da nuvem: {str(e)}")

def main():
    rclpy.init()
    node = PclToGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
