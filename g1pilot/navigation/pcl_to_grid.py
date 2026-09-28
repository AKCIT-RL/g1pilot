#!/usr/bin/env python3
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from mola_msgs.srv import RelocalizeNearPose
from rclpy.node import Node
import numpy as np
import subprocess
import rclpy
import glob
import math
import os

class PclToGridNode(Node):
    def __init__(self):
        """
            Declares all grid/localization parameters, sets up subscriptions/publisher, loads a
            MOLA map if one was given, and starts auto-relocalization if enabled.
        """
        super().__init__('pcl_to_grid')
        self.declare_parameter('resolution', 0.1)           # grid cell size, meters
        self.declare_parameter('width_m', 40.0)              # grid width, meters
        self.declare_parameter('height_m', 40.0)             # grid height, meters
        self.declare_parameter('min_z', 0.1)                 # obstacle height band, min meters
        self.declare_parameter('max_z', 1.0)                 # obstacle height band, max meters
        self.declare_parameter('mola_map', '')               # path to a MOLA .mm map to preload
        self.declare_parameter('min_points_per_cell', 3)      # min points to count as obstacle
        self.declare_parameter('min_obstacle_height', 0.08)   # min vertical extent per cell
        self.declare_parameter('auto_relocalize', True)
        self.declare_parameter('relocalize_x', 0.0)
        self.declare_parameter('relocalize_y', 0.0)
        self.declare_parameter('relocalize_yaw_deg', 0.0)

        self.res = self.get_parameter('resolution').value
        self.w_m = self.get_parameter('width_m').value
        self.h_m = self.get_parameter('height_m').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.mola_map_path = self.get_parameter('mola_map').value
        self.min_pts = self.get_parameter('min_points_per_cell').value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.auto_relocalize = self.get_parameter('auto_relocalize').value
        
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

        self._relocalize_sent = False
        if self.mola_map_path and self.auto_relocalize:
            self.relocalize_client = self.create_client(RelocalizeNearPose, '/relocalize_near_pose')
            self.relocalize_timer = self.create_timer(2.0, self._try_auto_relocalize)

    def _try_auto_relocalize(self):
        """
            Retries on a timer until /relocalize_near_pose is available, then calls it once with
            the configured relocalize_x/y/yaw_deg pose.
        """
        if self._relocalize_sent:
            self.relocalize_timer.cancel()
            return
        if not self.relocalize_client.service_is_ready():
            self.get_logger().info("Waiting for /relocalize_near_pose to auto-relocalize against the loaded map...")
            return
        x = self.get_parameter('relocalize_x').value
        y = self.get_parameter('relocalize_y').value
        yaw = math.radians(self.get_parameter('relocalize_yaw_deg').value)
        req = RelocalizeNearPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.header.stamp = self.get_clock().now().to_msg()
        req.pose.pose.pose.position.x = x
        req.pose.pose.pose.position.y = y
        req.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        req.pose.pose.covariance = [1.0 if i % 7 == 0 else 0.0 for i in range(36)]  # generous search radius
        self.get_logger().info(f"Auto-relocalizing near ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f} deg).")
        self.relocalize_client.call_async(req)
        self._relocalize_sent = True

    def cb_initialpose(self, msg):
        """
            Marks the robot as localized once a manual 2D Pose Estimate arrives.

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The estimated pose.
        """
        self.get_logger().info("Received initialpose. Robot is now localized!")
        self.localized = True

    def load_mola_map(self):
        """
            Converts the configured MOLA .mm map to PLY (via mm2ply) and bakes it into the
            static obstacle mask. No-op if mola_map wasn't set.
        """
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

            self.static_grid_mask = self.points_to_mask(pts)
            self.static_map_locked = True
            self.get_logger().info(f"Static millimeter map generated successfully with {np.sum(self.static_grid_mask)} obstacle cells!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to automatically load MOLA map: {str(e)}")

    def points_to_mask(self, pts):
        """
            Projects height-band-filtered points to the grid, keeping a cell as an obstacle only
            if BOTH:
              - it has at least min_points_per_cell points (rejects a single stray/noisy return), and
              - those points span at least min_obstacle_height of vertical extent (rejects a flat
                patch of ground).

            That second check is a grid-cell-local stand-in for "is the local surface normal
            pointing straight up" -- a true per-point normal would need a neighbor search (KD-tree +
            eigendecomposition per point), too expensive to run on every lidar sweep in a Python
            node at 10-20Hz. Since we already bin points into this same grid for the occupancy map,
            checking each cell's own Z range is a cheap, vectorized proxy for the same idea: a flat
            / near-horizontal patch (small Z spread) has a near-vertical normal (floor), while a
            patch with real vertical extent in a small footprint (a wall, a box edge, a leg) has a
            normal that's far from vertical. This is why min_z/max_z above can stay a loose sanity
            band instead of needing to precisely exclude the ground plane -- this is what actually
            rejects the floor now, including floor points that wobble to a slightly positive Z.
        """
        mask = np.zeros((self.height, self.width), dtype=bool)
        if pts.size == 0:
            return mask
        ix = ((pts[:, 0] - self.origin_x) / self.res).astype(int)
        iy = ((pts[:, 1] - self.origin_y) / self.res).astype(int)
        valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        ix, iy, z = ix[valid], iy[valid], pts[valid, 2]

        counts = np.zeros((self.height, self.width), dtype=np.int32)
        z_min = np.full((self.height, self.width), np.inf, dtype=np.float32)
        z_max = np.full((self.height, self.width), -np.inf, dtype=np.float32)
        np.add.at(counts, (iy, ix), 1)
        np.minimum.at(z_min, (iy, ix), z)
        np.maximum.at(z_max, (iy, ix), z)

        z_range = z_max - z_min
        mask[(counts >= self.min_pts) & (z_range >= self.min_obstacle_height)] = True
        return mask

    def load_ply_points(self, ply_path):
        """
            Reads XYZ points from a PLY file's ASCII vertex data.

        Args:
            ply_path (str): Path to the .ply file.

        Returns:
            np.ndarray: Nx3 array of points.
        """
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
        """
            Rebuilds the static obstacle mask from MOLA's live local map, until it gets locked
            (either by a preloaded mola_map, or once the robot moves away from its start position).

        Args:
            msg (sensor_msgs.msg.PointCloud2): MOLA's local map point cloud.
        """
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

            self.static_grid_mask = self.points_to_mask(pts)
            self.get_logger().info(f"Updated static map mask with {np.sum(self.static_grid_mask)} obstacle cells.")
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento do mapa local: {str(e)}")

    def cb_pose(self, msg):
        """
            Tracks localization state from MOLA's pose covariance, and detects/handles
            teleportation (relocalization) or the robot moving away from its start position.

        Args:
            msg (nav_msgs.msg.Odometry): MOLA's pose estimate, with covariance.
        """
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
        """
            Transforms the live lidar scan into the map frame, masks it into obstacle cells, and
            publishes the combined (static + live) occupancy grid.

        Args:
            msg (sensor_msgs.msg.PointCloud2): Live lidar point cloud.
        """
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

            live_mask = self.points_to_mask(pts_filtered)
            live_indices = np.where(live_mask) if np.any(live_mask) else None
            
            self.publish_grid(msg.header, live_indices)
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento do scan livox vivo: {str(e)}")

    def publish_grid(self, header, live_indices):
        """
            Builds and publishes the occupancy grid: the static mask (once localized) plus
            whatever live obstacle cells were just detected.

        Args:
            header (std_msgs.msg.Header): Header to stamp the grid with.
            live_indices (tuple[np.ndarray, np.ndarray] | None): (row, col) indices of live
                obstacle cells, or None if there are none this tick.
        """
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
    """
        Starts the pcl_to_grid ROS2 node and spins it until shutdown.
    """
    rclpy.init()
    node = PclToGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
