#!/usr/bin/env python3
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
import numpy as np
import subprocess
import rclpy
import glob
import os

class PclToGridNode(Node):
    def __init__(self):
        super().__init__('pcl_to_grid')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('width_m', 40.0)
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('min_z', 0.1)
        self.declare_parameter('max_z', 1.0)
        self.declare_parameter('mola_map', '')
        
        self.res = self.get_parameter('resolution').value
        self.w_m = self.get_parameter('width_m').value
        self.h_m = self.get_parameter('height_m').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.mola_map_path = self.get_parameter('mola_map').value
        
        self.width = int(self.w_m / self.res)
        self.height = int(self.h_m / self.res)
        self.origin_x = -self.w_m / 2.0
        self.origin_y = -self.h_m / 2.0
        
        # Static grid mask for the loaded millimeter map
        self.static_grid_mask = np.zeros((self.height, self.width), dtype=bool)
        self.static_map_locked = False
        self.localized = False
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Transient Local QoS Profile for static map topic
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Subscriptions
        self.sub_localmap = self.create_subscription(
            PointCloud2, 
            '/lidar_odometry/localmap_points', 
            self.cb_localmap, 
            qos_profile
        )
        self.sub_livox = self.create_subscription(
            PointCloud2, 
            '/livox/lidar', 
            self.cb_livox, 
            10
        )
        self.sub_pose = self.create_subscription(
            Odometry,
            '/lidar_odometry/pose',
            self.cb_pose,
            10
        )
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_initialpose,
            10
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.load_mola_map()
        self.get_logger().info(f"PCL to Grid initialized: {self.width}x{self.height} @ {self.res}m/px")

    def cb_initialpose(self, msg):
        self.get_logger().info("Received initialpose. Robot is now localized!")
        self.localized = True

    def load_mola_map(self):
        if not self.mola_map_path:
            self.get_logger().info("No mola_map parameter provided. Waiting for live localmap_points topic instead.")
            return
            
        self.get_logger().info(f"Loading MOLA map from: {self.mola_map_path}")
        
        # Clean old temp PLY files
        for f in glob.glob("/tmp/temp_map_*.ply"):
            try:
                os.remove(f)
            except Exception:
                pass
                
        try:
            # Setup environment copy
            env = os.environ.copy()
            
            # Run mm2ply command on the passed .mm file directly!
            cmd = ["mm2ply", "-i", self.mola_map_path, "-o", "/tmp/temp_map"]
            self.get_logger().info(f"Running command: {' '.join(cmd)}")
            result = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            self.get_logger().info(f"mm2ply output: {result.stdout}")
            self.get_logger().info(f"mm2ply stderr: {result.stderr}")
            
            # Find generated PLY files
            ply_files = glob.glob("/tmp/temp_map_*.ply") + glob.glob("/tmp/temp_map*.ply")
            ply_files = list(set(ply_files))
            
            all_pts = []
            
            # Load PLY files
            for pf in ply_files:
                self.get_logger().info(f"Parsing PLY layer file: {pf}")
                pts = self.load_ply_points(pf)
                if pts.size > 0:
                    all_pts.append(pts)
                    
            if not all_pts:
                self.get_logger().warn("No PLY points parsed from the MOLA map.")
                return
                
            pts = np.concatenate(all_pts, axis=0)
            self.get_logger().info(f"Successfully loaded {len(pts)} total points from MOLA .mm map.")
            
            # Filter points by height
            mask = (pts[:, 2] > self.min_z) & (pts[:, 2] < self.max_z)
            pts = pts[mask]
            
            # Project to static grid mask
            new_mask = np.zeros((self.height, self.width), dtype=bool)
            if pts.size > 0:
                ix = ((pts[:, 0] - self.origin_x) / self.res).astype(int)
                iy = ((pts[:, 1] - self.origin_y) / self.res).astype(int)
                
                valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
                new_mask[iy[valid], ix[valid]] = True
                
            self.static_grid_mask = new_mask
            self.static_map_locked = True
            self.get_logger().info(f"Static millimeter map generated successfully with {np.sum(self.static_grid_mask)} obstacle cells!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to automatically load MOLA map: {str(e)}")

    def load_ply_points(self, ply_path):
        points = []
        try:
            with open(ply_path, 'r') as f:
                in_header = True
                for line in f:
                    line = line.strip()
                    if in_header:
                        if line == "end_header":
                            in_header = False
                        continue
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            z = float(parts[2])
                            points.append([x, y, z])
                        except ValueError:
                            continue
        except Exception as e:
            self.get_logger().error(f"Error reading PLY file {ply_path}: {str(e)}")
        return np.array(points, dtype=np.float32)


    def cb_localmap(self, msg):
        if self.static_map_locked:
            return
            
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_list = [[p[0], p[1], p[2]] for p in points]
            if not pts_list:
                return
            
            pts = np.array(pts_list, dtype=np.float32)
            
            # Filtering by Z (height)
            mask = (pts[:, 2] > self.min_z) & (pts[:, 2] < self.max_z)
            pts = pts[mask]
            
            # Reset static mask
            new_mask = np.zeros((self.height, self.width), dtype=bool)
            
            if pts.size > 0:
                ix = ((pts[:, 0] - self.origin_x) / self.res).astype(int)
                iy = ((pts[:, 1] - self.origin_y) / self.res).astype(int)
                
                valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
                new_mask[iy[valid], ix[valid]] = True
                
            self.static_grid_mask = new_mask
            self.get_logger().info(f"Updated static map mask with {np.sum(self.static_grid_mask)} obstacle cells.")
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento do mapa local: {str(e)}")

    def cb_pose(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # Check MOLA pose covariance to detect localization
        # msg.pose.covariance is a 36-element array representing a 6x6 covariance matrix
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        std_x = np.sqrt(max(0.0, cov_x))
        std_y = np.sqrt(max(0.0, cov_y))
        
        # If the position uncertainty is below 1.0 meter, we are localized!
        if not self.localized:
            if std_x < 1.0 and std_y < 1.0:
                self.localized = True
                self.get_logger().info(f"Robot localized automatically! Covariance drops below 1.0m (std_x={std_x:.3f}m, std_y={std_y:.3f}m). Activating static millimeter map.")
        
        if not hasattr(self, 'start_x'):
            self.start_x = curr_x
            self.start_y = curr_y
            self.prev_x = curr_x
            self.prev_y = curr_y
            return
            
        # Detect teleportation (e.g. 2D Pose Estimate or relocalization convergence)
        dist_from_prev = np.hypot(curr_x - self.prev_x, curr_y - self.prev_y)
        if dist_from_prev > 1.0:
            self.get_logger().info(f"Robot teleported/aligned by {dist_from_prev:.2f}m. Resetting static map alignment...")
            self.start_x = curr_x
            self.start_y = curr_y
            if not self.mola_map_path:
                self.static_map_locked = False
            self.localized = True
            
        self.prev_x = curr_x
        self.prev_y = curr_y
        
        # Check if robot has moved away from the start position
        if not self.static_map_locked and not self.mola_map_path:
            dist_from_start = np.hypot(curr_x - self.start_x, curr_y - self.start_y)
            if dist_from_start > 0.5:
                self.static_map_locked = True
                self.localized = True
                self.get_logger().info(f"Robot moved {dist_from_start:.2f}m. Locking static map mask.")

    def cb_livox(self, msg):
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_list = [[p[0], p[1], p[2]] for p in points]
            if not pts_list:
                # If live scan is empty, publish static map only
                self.publish_grid(msg.header, None)
                return
            
            pts_sensor = np.array(pts_list, dtype=np.float32)
            
            # Look up transform from sensor frame to 'map'
            try:
                trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            except Exception as tf_ex:
                self.get_logger().warn(f"Failed to lookup transform map -> {msg.header.frame_id}: {str(tf_ex)}")
                # Fallback to publishing static map only
                self.publish_grid(msg.header, None)
                return
            
            # Extract translation and rotation
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            tz = trans.transform.translation.z
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            # Convert quaternion to rotation matrix
            R = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx**2 - 2*qy**2]
            ])
            T = np.array([tx, ty, tz])
            
            # Transform sensor points to map frame
            pts_map = np.dot(pts_sensor, R.T) + T
            
            # Filtering by Z (height) in map frame
            mask = (pts_map[:, 2] > self.min_z) & (pts_map[:, 2] < self.max_z)
            pts_filtered = pts_map[mask]
            
            live_indices = None
            if pts_filtered.size > 0:
                ix = ((pts_filtered[:, 0] - self.origin_x) / self.res).astype(int)
                iy = ((pts_filtered[:, 1] - self.origin_y) / self.res).astype(int)
                
                valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
                live_indices = (iy[valid], ix[valid])
            
            self.publish_grid(msg.header, live_indices)
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento do scan livox vivo: {str(e)}")

    def publish_grid(self, header, live_indices):
        # Create costmap grid starting from the permanent static mask (only if localized!)
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        if self.localized:
            grid[self.static_grid_mask] = 100
        
        # Add dynamic live indices
        if live_indices is not None:
            grid[live_indices[0], live_indices[1]] = 100
            
        # Build OccupancyGrid message
        occ = OccupancyGrid()
        occ.header = header
        occ.header.frame_id = 'map'
        occ.info.resolution = self.res
        occ.info.width = self.width
        occ.info.height = self.height
        occ.info.origin.position.x = self.origin_x
        occ.info.origin.position.y = self.origin_y
        occ.info.origin.orientation.w = 1.0
        occ.data = grid.flatten().tolist()
        
        self.pub.publish(occ)

def main():
    rclpy.init()
    node = PclToGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
