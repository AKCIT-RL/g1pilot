#!/usr/bin/env python3
import math, heapq
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Header, Bool
from std_srvs.srv import Trigger
from rclpy.duration import Duration

def _dist(a,b):
    """
        Euclidean distance between two (x, y) points.

    Args:
        a (tuple[float, float]): First point.
        b (tuple[float, float]): Second point.

    Returns:
        float: Distance between a and b.
    """
    dx=a[0]-b[0]; dy=a[1]-b[1]
    return math.hypot(dx,dy)

def _catmull_rom_centripetal(points, samples_per_seg=8, closed=False):
    """
        Smooths a polyline into a centripetal Catmull-Rom spline.

    Args:
        points (list[tuple[float, float]]): Control points, in order.
        samples_per_seg (int): Samples generated per input segment.
        closed (bool): Whether the spline wraps around into a closed loop.

    Returns:
        list[tuple[float, float]]: Smoothed points.
    """
    if len(points)<2: return points[:]
    P=points[:]
    P=([P[-1]]+P+[P[0],P[1]]) if closed else ([P[0]]+P+[P[-1]])
    out=[]
    for i in range(1,len(P)-2):
        p0,p1,p2,p3=P[i-1],P[i],P[i+1],P[i+2]
        t0=0.0
        t1=t0+math.sqrt(_dist(p0,p1))
        t2=t1+math.sqrt(_dist(p1,p2))
        t3=t2+math.sqrt(_dist(p2,p3))
        if t1==t0 or t2==t1 or t3==t2:
            if not out or _dist(out[-1],p1)>1e-6: out.append(p1)
            if i==len(P)-3 and (not out or _dist(out[-1],p2)>1e-6): out.append(p2)
            continue
        for s in range(samples_per_seg):
            t=t1+(t2-t1)*s/float(samples_per_seg)
            A1=((t1-t)/(t1-t0))*p0[0]+((t-t0)/(t1-t0))*p1[0], ((t1-t)/(t1-t0))*p0[1]+((t-t0)/(t1-t0))*p1[1]
            A2=((t2-t)/(t2-t1))*p1[0]+((t-t1)/(t2-t1))*p2[0], ((t2-t)/(t2-t1))*p1[1]+((t-t1)/(t2-t1))*p2[1]
            A3=((t3-t)/(t3-t2))*p2[0]+((t-t2)/(t3-t2))*p3[0], ((t3-t)/(t3-t2))*p2[1]+((t-t2)/(t3-t2))*p3[1]
            B1=((t2-t)/(t2-t0))*A1[0]+((t-t0)/(t2-t0))*A2[0], ((t2-t)/(t2-t0))*A1[1]+((t-t0)/(t2-t0))*A2[1]
            B2=((t3-t)/(t3-t1))*A2[0]+((t-t1)/(t3-t1))*A3[0], ((t3-t)/(t3-t1))*A2[1]+((t-t1)/(t3-t1))*A3[1]
            C=((t2-t)/(t2-t1))*B1[0]+((t-t1)/(t2-t1))*B2[0], ((t2-t)/(t2-t1))*B1[1]+((t-t1)/(t2-t1))*B2[1]
            if not out or _dist(out[-1],C)>1e-6: out.append(C)
        if i==len(P)-3:
            if not out or _dist(out[-1],p2)>1e-6: out.append(p2)
    return out

class DijkstraPlanner(Node):
    def __init__(self):
        """
            Declares all navigation parameters, sets up subscriptions/publishers/timers, and
            initializes the planner's map/pose/mission state.
        """
        super().__init__('dijkstra_planner')
        self.declare_parameter('map_topic','/map')                        # occupancy grid topic
        self.declare_parameter('odom_topic','/lidar_odometry/pose_fixed')  # robot pose topic
        self.declare_parameter('goal_topic','/g1pilot/goal')               # incoming goal topic
        self.declare_parameter('path_topic','/g1pilot/path')               # published path topic
        self.declare_parameter('occ_threshold',50)          # occupancy value counted as obstacle
        self.declare_parameter('allow_diagonal',True)        # unused (kept for compatibility)
        self.declare_parameter('straight_steps',50)          # unused (kept for compatibility)
        self.declare_parameter('inflation_radius_m',0.40)    # obstacle safety margin, meters
        self.declare_parameter('smooth_enable',True)         # unused (kept for compatibility)
        self.declare_parameter('smooth_samples_per_segment',8)  # unused (kept for compatibility)
        self.declare_parameter('smooth_closed',False)        # unused (kept for compatibility)
        self.declare_parameter('simplify_min_dist',0.02)     # unused (kept for compatibility)
        self.declare_parameter('shortcut_enable',True)       # unused (kept for compatibility)
        self.declare_parameter('turn_cost_gain',2.0)         # Dijkstra's penalty for sharp turns
        qos=QoSProfile(depth=10)
        self.sub_map=self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.cb_map,qos)
        self.sub_odom=self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.cb_odom,qos)
        self.sub_goal=self.create_subscription(PoseStamped,self.get_parameter('goal_topic').value,self.cb_goal,qos)
        self.pub_path=self.create_publisher(Path,self.get_parameter('path_topic').value,qos)
        self.pub_inflated=self.create_publisher(OccupancyGrid,'/dijkstra_planner/inflated_map',qos)

        self.declare_parameter('goal_tolerance', 0.20)  # position tolerance to the goal, meters
        self.declare_parameter('goal_orientation_tolerance_deg', 8.0)
        self.declare_parameter('replan_cooldown', 2.0)  # min seconds between blockage replans
        self.declare_parameter('check_rate', 2.0)       # Hz, how often to check nav status
        self.declare_parameter('recovery_sleep_duration', 3.0)  # seconds between recovery tries
        self.declare_parameter('max_recovery_attempts', 3)      # tries before giving up a goal
        self.declare_parameter('check_horizon_dist', 1.0)  # meters ahead checked for blockage
        self.declare_parameter('shortcut_check_interval_s', 5.0)      # how often to look for a shortcut
        self.declare_parameter('shortcut_min_improvement_pct', 15.0)  # min improvement to switch
        self.declare_parameter('debug', False)  # verbose per-tick path-check logging
        self.declare_parameter('check_start_dist', 0.15)  # safety margin behind the robot on path
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw_deg', 0.0)
        self.declare_parameter('geofence_points', [0.0])
        self.declare_parameter('geofence_check_interval_s', 1.0)    # seconds between checks
        self.declare_parameter('geofence_resend_interval_s', 3.0)   # retry interval once outside
        self.declare_parameter('pose_publish_rate_hz', 10.0)  # Hz for /g1pilot/robot_pose
        self.debug = self.get_parameter('debug').value
        self._last_geofence_send = None

        self.goal_queue = []
        self.active_goal = None
        self.current_path_points = []
        self.last_replan_time = self.get_clock().now()

        # Recovery state
        self.in_recovery = False
        self.recovery_attempts = 0
        self.next_recovery_time = None

        # Sticky "we've arrived at the goal position" latch -- see check_navigation_status.
        self.position_reached = False

        self.sub_clear = self.create_subscription(
            Bool,
            '/g1pilot/clear_goals',
            self.cb_clear_goals,
            qos
        )
        self.srv_go_home = self.create_service(Trigger, '/g1pilot/go_home', self.cb_go_home)
        self.pub_robot_pose = self.create_publisher(PoseStamped, '/g1pilot/robot_pose', qos)

        self.timer = self.create_timer(
            1.0 / self.get_parameter('check_rate').value,
            self.check_navigation_status
        )
        self.shortcut_timer = self.create_timer(
            self.get_parameter('shortcut_check_interval_s').value,
            self.check_for_shorter_path
        )
        self.geofence_timer = self.create_timer(
            self.get_parameter('geofence_check_interval_s').value,
            self.check_geofence
        )
        self.pose_pub_timer = self.create_timer(
            1.0 / self.get_parameter('pose_publish_rate_hz').value,
            self.publish_robot_pose
        )

        self.map=None
        self.map_frame='map'
        self.res=self.ox=self.oy=0.0
        self.w=self.h=0
        self.occ=[]; self.occ_inf=[]
        self.inf_radius_cells=0
        self.have_pose=False
        self.px=self.py=self.pyaw=0.0

    def cb_map(self,msg):
        """
            Stores the latest occupancy grid and recomputes its inflated version.

        Args:
            msg (nav_msgs.msg.OccupancyGrid): Latest map.
        """
        self.map=msg
        self.map_frame=msg.header.frame_id or 'map'
        self.res=float(msg.info.resolution)
        self.ox=float(msg.info.origin.position.x)
        self.oy=float(msg.info.origin.position.y)
        self.w=int(msg.info.width)
        self.h=int(msg.info.height)
        self.occ=list(msg.data)
        # Use parameter for inflation
        radius = self.get_parameter('inflation_radius_m').value
        self.inf_radius_cells=int(math.ceil(radius/self.res)) if self.res>0.0 else 0
        self.get_logger().debug(f"Inflating map with radius {radius}m ({self.inf_radius_cells} cells)")
        self.occ_inf=self.inflate_occupancy(self.occ,self.w,self.h,self.inf_radius_cells,50)
        
        # Publish inflated map for visualization
        occ_inf_msg = OccupancyGrid()
        occ_inf_msg.header = msg.header
        occ_inf_msg.info = msg.info
        occ_inf_msg.data = [int(v) for v in self.occ_inf]
        self.pub_inflated.publish(occ_inf_msg)

    def cb_odom(self,msg):
        """
            Updates the robot's tracked position and yaw from odometry.

        Args:
            msg (nav_msgs.msg.Odometry): Latest odometry reading.
        """
        self.px=float(msg.pose.pose.position.x)
        self.py=float(msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        siny_cosp=2*(q.w*q.z+q.x*q.y)
        cosy_cosp=1-2*(q.y*q.y+q.z*q.z)
        self.pyaw=math.atan2(siny_cosp,cosy_cosp)
        self.have_pose=True

    def cb_goal(self, msg):
        """
            Handles a new navigation goal: snaps it out of any obstacle buffer once, then
            replaces the active mission with it.

        Args:
            msg (geometry_msgs.msg.PoseStamped): Requested goal pose.
        """
        if not self.have_pose:
            self.get_logger().warn("No odom pose yet. Cannot set goal.")
            return

        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)

        if self.map is not None:
            gx_i, gy_i = self.world_to_grid(gx, gy)
            if self.in_bounds(gx_i, gy_i) and self.is_raw_occ(gx_i, gy_i):
                self.get_logger().error(f"Goal ({gx:.2f}, {gy:.2f}) is on top of an actual obstacle -- rejected.")
                return
            if self.in_bounds(gx_i, gy_i) and self.is_occ(gx_i, gy_i):
                snapped = self.find_nearest_free_cell(gx_i, gy_i)
                if snapped is None:
                    self.get_logger().error(
                        f"Goal ({gx:.2f}, {gy:.2f}) is inside an obstacle's safety buffer and no "
                        f"free cell was found nearby -- rejected."
                    )
                    return
                sx, sy = self.grid_to_world(*snapped)
                self.get_logger().info(
                    f"Goal ({gx:.2f}, {gy:.2f}) is inside an obstacle's safety buffer -- "
                    f"snapping to the nearest free point ({sx:.2f}, {sy:.2f}) instead."
                )
                msg.pose.position.x, msg.pose.position.y = sx, sy
                gx, gy = sx, sy

        # Replaces the current mission rather than appending to it.
        self.get_logger().info(f"Received new goal: ({gx:.2f}, {gy:.2f}). Replacing current goal.")
        self.goal_queue = [msg]
        self.process_next_goal()

    def cb_clear_goals(self, msg):
        """
            Clears the goal queue and stops navigation when triggered.

        Args:
            msg (std_msgs.msg.Bool): True to clear; ignored otherwise.
        """
        if msg.data:
            self.goal_queue.clear()
            self.active_goal = None
            self.current_path_points = []
            
            # Reset recovery state
            self.in_recovery = False
            self.recovery_attempts = 0
            self.next_recovery_time = None
            self.position_reached = False

            # Publish empty path to signal stop to nav2point
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = self.map_frame
            self.pub_path.publish(empty_path)
            self.get_logger().info("Cleared all mission goals and stopped navigation.")

    def send_robot_home(self):
        """
            Builds the fixed home_x/home_y/home_yaw_deg pose and feeds it through cb_goal, as if
            it had arrived on /goal_pose -- shared by the /g1pilot/go_home service and the geofence
            check below, so both trigger the exact same snapping/queueing behavior.
        """
        hx = self.get_parameter('home_x').value
        hy = self.get_parameter('home_y').value
        hyaw = math.radians(self.get_parameter('home_yaw_deg').value)
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = hx
        msg.pose.position.y = hy
        msg.pose.orientation.z = math.sin(hyaw / 2.0)
        msg.pose.orientation.w = math.cos(hyaw / 2.0)
        self.get_logger().info(f"Sending robot home ({hx:.2f}, {hy:.2f}).")
        self.cb_goal(msg)

    def cb_go_home(self, request, response):
        """
            Trigger service, no request fields at all -- calling it with nothing sends the robot
            to the fixed home pose (see send_robot_home).
        """
        self.send_robot_home()
        response.success = True
        response.message = f"Sending robot home ({self.get_parameter('home_x').value:.2f}, {self.get_parameter('home_y').value:.2f})."
        return response

    def _point_in_polygon(self, x, y, poly):
        """
            Standard ray-casting point-in-polygon test -- works for any simple polygon (not just
            convex ones), given its vertices in perimeter order. Counts how many edges a horizontal
            ray cast from (x,y) crosses; odd count means inside.
        """
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def check_geofence(self):
        """
            Runs on its own timer regardless of any active goal -- the geofence applies whether
            the robot is navigating, idle, or mid-mission. While the robot is outside the polygon,
            retries roughly every geofence_resend_interval_s -- not just once on the way out -- so a
            failed first attempt (planning failure, etc.) keeps getting retried instead of leaving
            the robot stranded outside forever. But it skips the retry if a mission toward home is
            already active: resending the same target while nav2point is mid-walk would replace the
            path and reset its align-then-drive state every cycle (cb_goal always treats a new goal
            as "go here now"), never letting it settle -- the same repeated-restart problem fixed
            earlier for ordinary navigation. Only escalates when nothing is actively heading home.
        """
        pts_flat = self.get_parameter('geofence_points').value
        if not self.have_pose or not pts_flat or len(pts_flat) < 6:
            return
        poly = [(pts_flat[i], pts_flat[i + 1]) for i in range(0, len(pts_flat) - 1, 2)]
        if self._point_in_polygon(self.px, self.py, poly):
            self._last_geofence_send = None
            return

        now = self.get_clock().now()
        if self._last_geofence_send is not None:
            elapsed = (now - self._last_geofence_send).nanoseconds / 1e9
            if elapsed < self.get_parameter('geofence_resend_interval_s').value:
                return
        self._last_geofence_send = now

        hx = self.get_parameter('home_x').value
        hy = self.get_parameter('home_y').value
        heading_home = (
            self.active_goal is not None
            and math.hypot(float(self.active_goal.pose.position.x) - hx,
                            float(self.active_goal.pose.position.y) - hy) <= self.get_parameter('goal_tolerance').value
        )
        if heading_home:
            return
        self.get_logger().warn(f"Robot outside the geofence at ({self.px:.2f}, {self.py:.2f}) -- sending it home.")
        self.send_robot_home()

    def publish_robot_pose(self):
        """
            Publishes the robot's current pose on /g1pilot/robot_pose.
        """
        if not self.have_pose:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.px
        msg.pose.position.y = self.py
        msg.pose.orientation.z = math.sin(self.pyaw / 2.0)
        msg.pose.orientation.w = math.cos(self.pyaw / 2.0)
        self.pub_robot_pose.publish(msg)

    def process_next_goal(self):
        """
            Pops the next goal off the queue and starts planning to it, or stops navigation if
            the queue is empty.
        """
        self.in_recovery = False
        self.recovery_attempts = 0
        self.next_recovery_time = None
        self.position_reached = False

        if not self.goal_queue:
            self.active_goal = None
            self.current_path_points = []
            
            # Publish empty path to stop robot
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = self.map_frame
            self.pub_path.publish(empty_path)
            self.get_logger().info("No more goals in queue. Navigation stopped.")
            return

        self.active_goal = self.goal_queue.pop(0)
        gx = float(self.active_goal.pose.position.x)
        gy = float(self.active_goal.pose.position.y)
        self.get_logger().info(f"Processing next goal: ({gx:.2f}, {gy:.2f}). Remaining in queue: {len(self.goal_queue)}")
        
        # Plan path immediately
        self.plan_path_to_active_goal()

    def _compute_path(self):
        """
            Runs Dijkstra + smoothing from the robot's current position to the active goal and
            returns the resulting waypoint list, or None if no path exists. Pure computation -- no
            recovery-state changes, no publishing -- so both plan_path_to_active_goal and the
            periodic shortcut check (check_for_shorter_path) can build on it.
        """
        if self.active_goal is None or not self.have_pose or self.map is None:
            return None

        gx = float(self.active_goal.pose.position.x)
        gy = float(self.active_goal.pose.position.y)

        sx, sy = self.world_to_grid(self.px, self.py)
        gx_i, gy_i = self.world_to_grid(gx, gy)

        if not self.in_bounds(sx, sy) or not self.in_bounds(gx_i, gy_i):
            return None
        
        if self.is_raw_occ(sx, sy):
            return None

        # Temporarily un-inflate an escape route if the start is inside a buffer.
        escape_cells = self.compute_inflation_escape(sx, sy)
        saved_inflation = {}
        for (ex, ey) in escape_cells:
            idx = ey * self.w + ex
            if self.occ_inf[idx] != self.occ[idx]:
                saved_inflation[idx] = self.occ_inf[idx]
                self.occ_inf[idx] = self.occ[idx]
        
        try:
            path_idx = self.dijkstra((sx, sy, self.pyaw), (gx_i, gy_i))
            if not path_idx:
                return None

            pts = [self.grid_to_world(ix, iy) for ix, iy in path_idx]
            pts = self.simplify_spacing(pts, 0.02)
            pts = self.shortcut_path(pts)  # every consecutive pair already verified clear
            smoothed = _catmull_rom_centripetal(pts, 8, False)

            smoothed_ok = all(
                self.grid_line_clear(self.world_to_grid(*smoothed[i]), self.world_to_grid(*smoothed[i + 1]))
                for i in range(len(smoothed) - 1)
            )
            final_pts = smoothed if smoothed_ok else pts
        finally:
            for idx, v in saved_inflation.items():
                self.occ_inf[idx] = v

        return final_pts

    def plan_path_to_active_goal(self):
        """
            Plans and publishes a path to the active goal, entering recovery on failure.

        Returns:
            bool: True if a path was successfully planned and published.
        """
        if self.active_goal is None:
            return False
        if not self.have_pose:
            self.get_logger().warn("No odom pose yet. Cannot plan path.")
            return False
        if self.map is None:
            self.get_logger().error("No occupancy map received yet!")
            self.handle_planning_failure()
            return False

        gx = float(self.active_goal.pose.position.x)
        gy = float(self.active_goal.pose.position.y)
        gq = self.active_goal.pose.orientation
        goal_yaw = math.atan2(2.0 * (gq.w * gq.z + gq.x * gq.y), 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z))

        sx, sy = self.world_to_grid(self.px, self.py)
        gx_i, gy_i = self.world_to_grid(gx, gy)
        if not self.in_bounds(sx, sy) or not self.in_bounds(gx_i, gy_i):
            self.get_logger().error(f"Planning failed: Start ({self.px:.2f}, {self.py:.2f}) or Goal ({gx:.2f}, {gy:.2f}) is out of map bounds!")
            self.handle_planning_failure()
            return False
        if self.is_raw_occ(sx, sy):
            self.get_logger().error(f"Planning failed: Start ({self.px:.2f}, {self.py:.2f}) is occupied!")
            self.handle_planning_failure()
            return False

        final_pts = self._compute_path()
        if final_pts is None:
            self.get_logger().error(f"Planning failed: No path found by Dijkstra to goal ({gx:.2f}, {gy:.2f})!")
            self.handle_planning_failure()
            return False

        self.current_path_points = final_pts
        self.publish_path(final_pts, self.map_frame, final_yaw=goal_yaw)

        # Successfully planned path, reset recovery status if it was in recovery
        if self.in_recovery:
            self.get_logger().info("Recovery succeeded! Path replanned successfully.")
            self.in_recovery = False
            self.recovery_attempts = 0
            self.next_recovery_time = None

        return True

    def path_length_m(self, pts):
        """
            Total length of a polyline.

        Args:
            pts (list[tuple[float, float]]): Points, in order.

        Returns:
            float: Sum of consecutive segment lengths.
        """
        if len(pts) < 2:
            return 0.0
        return sum(_dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    def remaining_path_length_m(self, pts):
        """
            Distance the robot still has to walk along pts (in order) if it keeps following
            them, measured from the robot's live position -- not from whichever waypoint happens to
            be nearest. On a short/unsimplified path (often just [start, goal] after shortcut_path
            collapses a clear line), the nearest *waypoint* can be well behind the robot's actual
            progress along that line, which either overstates or (if naively added to the segment
            length) double-counts the distance already walked. This instead projects the robot's
            position onto whichever segment it's actually closest to (clamped to that segment, so it
            never projects past an endpoint), and sums from there: the leftover length on that
            segment, plus every segment after it.
        """
        if len(pts) < 2:
            return 0.0
        best_dist = float('inf')
        best_remaining = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg_dx, seg_dy = bx - ax, by - ay
            seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
            t = 0.0
            if seg_len2 > 1e-9:
                t = max(0.0, min(1.0, ((self.px - ax) * seg_dx + (self.py - ay) * seg_dy) / seg_len2))
            proj_x, proj_y = ax + t * seg_dx, ay + t * seg_dy
            d = math.hypot(self.px - proj_x, self.py - proj_y)
            if d < best_dist:
                best_dist = d
                best_remaining = math.hypot(bx - proj_x, by - proj_y) + self.path_length_m(pts[i + 1:])
        return best_remaining

    def check_for_shorter_path(self):
        """
            Runs on its own timer (shortcut_check_interval_s), independent of the reactive
            blockage check in check_navigation_status. That one only replans when the current path
            is actually obstructed; this one looks for a route that's simply *better* than what
            we're already walking -- e.g. a shortcut that opened up because an obstacle moved or a
            map update cleared some cells. Switches only if the freshly-computed path beats however
            much of the current one is left to walk by at least shortcut_min_improvement_pct, so
            minor noise in the replanned route doesn't cause constant path-switching.
        """
        if self.active_goal is None or self.in_recovery or self.position_reached:
            return
        if not self.current_path_points or not self.have_pose or self.map is None:
            return

        sx, sy = self.world_to_grid(self.px, self.py)
        if self.in_bounds(sx, sy) and self.is_occ(sx, sy) and not self.is_raw_occ(sx, sy):
            return

        current_remaining = self.remaining_path_length_m(self.current_path_points)
        if current_remaining <= 0.0:
            return

        candidate = self._compute_path()
        if candidate is None:
            return
        candidate_len = self.path_length_m(candidate)

        min_improvement = self.get_parameter('shortcut_min_improvement_pct').value / 100.0
        if candidate_len <= current_remaining * (1.0 - min_improvement):
            self.get_logger().info(
                f"Found a shorter path to goal ({candidate_len:.2f}m vs {current_remaining:.2f}m "
                f"remaining on the current one) -- switching to it."
            )
            self.current_path_points = candidate
            gq = self.active_goal.pose.orientation
            goal_yaw = math.atan2(2.0 * (gq.w * gq.z + gq.x * gq.y), 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z))
            self.publish_path(candidate, self.map_frame, final_yaw=goal_yaw)

    def handle_planning_failure(self):
        """
            Stops the robot, and either schedules another recovery attempt or gives up on the
            active goal once max_recovery_attempts is exceeded.
        """
        self.current_path_points = []
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = self.map_frame
        self.pub_path.publish(empty_path)

        max_attempts = self.get_parameter('max_recovery_attempts').value
        sleep_duration = self.get_parameter('recovery_sleep_duration').value

        if not self.in_recovery:
            self.in_recovery = True
            self.recovery_attempts = 1
        else:
            self.recovery_attempts += 1

        if self.recovery_attempts > max_attempts:
            self.get_logger().error(f"Max recovery attempts ({max_attempts}) exceeded. Discarding active goal.")
            self.in_recovery = False
            self.recovery_attempts = 0
            self.next_recovery_time = None
            self.process_next_goal()
        else:
            self.get_logger().info(f"Scheduling recovery attempt {self.recovery_attempts}/{max_attempts} in {sleep_duration} seconds.")
            self.next_recovery_time = self.get_clock().now() + Duration(seconds=sleep_duration)

    def check_navigation_status(self):
        """
            Runs on check_rate: drives the recovery retry timer, detects goal arrival, and
            detects (and reacts to) a blocked path.
        """
        if self.active_goal is None:
            return
        if not self.have_pose:
            return

        if self.in_recovery:
            if self.next_recovery_time is not None and self.get_clock().now() >= self.next_recovery_time:
                self.next_recovery_time = None
                self.get_logger().info(f"Executing recovery attempt {self.recovery_attempts}/{self.get_parameter('max_recovery_attempts').value}...")
                self.plan_path_to_active_goal()
            return

        # Check if goal is reached
        gx = float(self.active_goal.pose.position.x)
        gy = float(self.active_goal.pose.position.y)
        dist_to_goal = math.hypot(gx - self.px, gy - self.py)

        goal_tol = self.get_parameter('goal_tolerance').value

        if not self.position_reached and dist_to_goal <= goal_tol:
            self.position_reached = True

        if self.position_reached:
            gq = self.active_goal.pose.orientation
            goal_yaw = math.atan2(2.0 * (gq.w * gq.z + gq.x * gq.y), 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z))
            yaw_err = math.atan2(math.sin(goal_yaw - self.pyaw), math.cos(goal_yaw - self.pyaw))
            orientation_tol = math.radians(self.get_parameter('goal_orientation_tolerance_deg').value)
            if abs(yaw_err) > orientation_tol:
                return  # still rotating to the goal's own heading; not done yet
            self.get_logger().info(f"Goal ({gx:.2f}, {gy:.2f}) reached! Proceeding to next goal.")
            self.process_next_goal()
            return

        if not self.current_path_points:
            self.plan_path_to_active_goal()
            return

        # Find closest point on path to the robot.
        min_dist = float('inf')
        closest_idx = 0
        for i, (x, y) in enumerate(self.current_path_points):
            d = math.hypot(x - self.px, y - self.py)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Back up check_start_dist meters from the closest point before checking (safety margin).
        check_start_dist = self.get_parameter('check_start_dist').value
        start_idx = closest_idx
        remaining = check_start_dist
        while start_idx > 0 and remaining > 0.0:
            remaining -= _dist(self.current_path_points[start_idx - 1], self.current_path_points[start_idx])
            start_idx -= 1

        # Densify the path points starting from start_idx.
        dense_points = []
        for i in range(start_idx, len(self.current_path_points) - 1):
            p1 = self.current_path_points[i]
            p2 = self.current_path_points[i+1]
            seg_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            step = 0.05
            num_steps = int(math.ceil(seg_dist / step))
            if num_steps <= 0:
                dense_points.append(p1)
            else:
                for s in range(num_steps):
                    alpha = s / float(num_steps)
                    x = (1 - alpha) * p1[0] + alpha * p2[0]
                    y = (1 - alpha) * p1[1] + alpha * p2[1]
                    dense_points.append((x, y))
        if self.current_path_points:
            dense_points.append(self.current_path_points[-1])

        # Check for obstacles along the dense path points.
        check_horizon_dist = self.get_parameter('check_horizon_dist').value
        path_blocked = False
        if self.debug:
            self.get_logger().info(f"--- PATH CHECK START --- Robot pos: ({self.px:.2f}, {self.py:.2f})")
        for i, (x, y) in enumerate(dense_points):
            dist_to_robot = math.hypot(x - self.px, y - self.py)

            # Stop checking if we exceed the horizon distance
            if dist_to_robot > check_horizon_dist:
                if self.debug:
                    self.get_logger().info(f"Point {i}: ({x:.2f}, {y:.2f}) exceeds horizon ({dist_to_robot:.2f}m > {check_horizon_dist}m). Stopping check.")
                break

            ix, iy = self.world_to_grid(x, y)
            in_b = self.in_bounds(ix, iy)
            val = self.occ[iy*self.w+ix] if in_b else -1

            # Log only every 10th point to avoid terminal flooding, or if it is an obstacle
            if self.debug and (i % 10 == 0 or (in_b and self.is_raw_occ(ix, iy))):
                self.get_logger().info(f"Point {i}: ({x:.2f}, {y:.2f}) -> dist={dist_to_robot:.2f}m, grid=({ix}, {iy}), val={val}, is_raw_occ={self.is_raw_occ(ix, iy) if in_b else False}")

            if in_b and self.is_raw_occ(ix, iy):
                path_blocked = True
                break

        if path_blocked:
            now = self.get_clock().now()
            elapsed = (now - self.last_replan_time).nanoseconds / 1e9
            if elapsed >= self.get_parameter('replan_cooldown').value:
                self.get_logger().warn(f"Path blockage detected! Replanning path to active goal ({gx:.2f}, {gy:.2f}).")
                self.plan_path_to_active_goal()
                self.last_replan_time = now

    def world_to_grid(self,x,y):
        """
            Converts world coordinates to grid indices.

        Args:
            x (float): World X.
            y (float): World Y.

        Returns:
            tuple[int, int]: Grid (ix, iy).
        """
        return int(math.floor((x-self.ox)/self.res)), int(math.floor((y-self.oy)/self.res))

    def grid_to_world(self,ix,iy):
        """
            Converts grid indices to world coordinates (cell center).

        Args:
            ix (int): Grid column.
            iy (int): Grid row.

        Returns:
            tuple[float, float]: World (x, y).
        """
        return self.ox+(ix+0.5)*self.res, self.oy+(iy+0.5)*self.res

    def in_bounds(self,ix,iy):
        """
            Checks whether a grid cell is within the map.

        Args:
            ix (int): Grid column.
            iy (int): Grid row.

        Returns:
            bool: True if (ix, iy) is inside the map.
        """
        return 0<=ix<self.w and 0<=iy<self.h

    def is_occ(self,ix,iy):
        """
            Checks the inflated occupancy grid (raw obstacles plus their safety buffer).

        Args:
            ix (int): Grid column.
            iy (int): Grid row.

        Returns:
            bool: True if the cell is occupied or buffered.
        """
        v=self.occ_inf[iy*self.w+ix]
        return v>=50 and v!=255

    def is_raw_occ(self,ix,iy):
        """
            Checks the raw occupancy grid (actual detected obstacles only).

        Args:
            ix (int): Grid column.
            iy (int): Grid row.

        Returns:
            bool: True if the cell is an actual obstacle.
        """
        v=self.occ[iy*self.w+ix]
        return v>=50 and v!=255

    def find_nearest_free_cell(self, cx, cy):
        """
            BFS out from (cx,cy) (breadth-first, so the first hit is genuinely the nearest by
            grid-step distance) for the closest cell that's fully free -- not just off the raw
            obstacle, off its inflation buffer too. Used once, at mission-start, to snap a requested
            goal sitting inside a buffer to someplace actually safe to stand at. Returns None if the
            whole reachable map is occupied (shouldn't happen in practice).
        """
        if not self.in_bounds(cx, cy):
            return None
        if not self.is_occ(cx, cy):
            return (cx, cy)
        visited = {(cx, cy)}
        q = deque([(cx, cy)])
        while q:
            x, y = q.popleft()
            for dx, dy in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)):
                nx, ny = x+dx, y+dy
                if not self.in_bounds(nx, ny) or (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                if not self.is_occ(nx, ny):
                    return (nx, ny)
                q.append((nx, ny))
        return None

    def compute_inflation_escape(self, sx, sy):
        """
            Shortest-path BFS out from (sx,sy) over cells that are only inflated-occupied (inside
            another obstacle's safety buffer), never crossing an actual (raw) obstacle cell. Stops as
            soon as it reaches ANY genuinely free cell and returns only the cells on that one
            shortest route -- not the whole connected inflated region the start happens to sit in,
            which can be large (overlapping buffers near a tight passage, or a generous
            inflation_radius_m) and would let Dijkstra route through far more of the safety margin
            than is actually needed just to get the robot unstuck. (The previous version used
            frontier.pop(), a stack/LIFO despite the docstring saying BFS, and kept expanding through
            the *entire* reachable inflated area instead of stopping at the first way out.)
        """
        if not self.is_occ(sx, sy) or self.is_raw_occ(sx, sy):
            return set()
        visited = {(sx, sy)}
        parent = {}
        q = deque([(sx, sy)])
        while q:
            x, y = q.popleft()
            for dx, dy in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)):
                nx, ny = x+dx, y+dy
                if not self.in_bounds(nx, ny) or (nx, ny) in visited or self.is_raw_occ(nx, ny):
                    continue
                visited.add((nx, ny))
                parent[(nx, ny)] = (x, y)
                if not self.is_occ(nx, ny):
                    escape = {(nx, ny)}
                    cur = (nx, ny)
                    while cur != (sx, sy):
                        cur = parent[cur]
                        escape.add(cur)
                    return escape
                q.append((nx, ny))
        
        return visited

    def neighbors(self,ix,iy):
        """
            Yields free grid neighbors of a cell for Dijkstra: cardinal directions at cost 1,
            diagonals at cost sqrt(2) (skipped if it would cut through an obstacle's corner).

        Args:
            ix (int): Cell column.
            iy (int): Cell row.

        Yields:
            tuple[int, int, float]: (neighbor x, neighbor y, step cost).
        """
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx,ny=ix+dx,iy+dy
            if self.in_bounds(nx,ny) and not self.is_occ(nx,ny):
                yield nx,ny,1.0
        rt2=math.sqrt(2)
        for dx,dy in [(-1,-1),(1,-1),(-1,1),(1,1)]:
            nx,ny=ix+dx,iy+dy
            if self.in_bounds(nx,ny) and not self.is_occ(nx,ny):
                if not self.is_occ(ix+dx, iy) and not self.is_occ(ix, iy+dy):
                    yield nx,ny,rt2

    def dijkstra(self,start,goal):
        """
            Finds the lowest-cost grid path from start to goal, penalizing sharp turns.

        Args:
            start (tuple[int, int, float]): (x, y, initial heading).
            goal (tuple[int, int]): Target cell.

        Returns:
            list[tuple[int, int]] | None: Path cells from start to goal, or None if unreachable.
        """
        sx,sy,syaw=start; gx,gy=goal
        dist={(sx,sy):0.0}; prev={}
        pq=[(0.0,sx,sy,syaw)]
        vis=set()
        k_turn=float(self.get_parameter('turn_cost_gain').value)
        while pq:
            d,x,y,yaw_prev=heapq.heappop(pq)
            if (x,y) in vis: continue
            vis.add((x,y))
            if (x,y)==(gx,gy): break
            for nx,ny,c in self.neighbors(x,y):
                yaw_next=math.atan2(ny-y,nx-x)
                delta=abs(math.atan2(math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev)))
                turn_cost=1.0 + k_turn * delta
                nd=d + c * turn_cost
                if nd < dist.get((nx,ny),float('inf')):
                    dist[(nx,ny)]=nd
                    prev[(nx,ny)]=(x,y,yaw_next)
                    heapq.heappush(pq,(nd,nx,ny,yaw_next))
        if (gx,gy) not in dist: return None
        path=[]; cur=(gx,gy)
        while cur in prev or cur==(sx,sy):
            path.append(cur)
            if cur==(sx,sy): break
            cur=(prev[cur][0],prev[cur][1])
        path.reverse()
        return path

    def publish_path(self,pts,frame_id,final_yaw=None):
        """
            Builds and publishes a Path message from a list of points, with a smoothed
            direction-of-travel heading at each point except the last, which uses final_yaw as-is.

        Args:
            pts (list[tuple[float, float]]): Path points, in order.
            frame_id (str): Frame the path is expressed in.
            final_yaw (float | None): Explicit heading for the last point, if any.
        """
        path=Path()
        path.header=Header()
        path.header.stamp=self.get_clock().now().to_msg()
        path.header.frame_id=frame_id
        path.poses=[]
        prev_yaw=self.pyaw
        for i,(x,y) in enumerate(pts):
            p=PoseStamped()
            p.header=path.header
            p.pose.position.x=x; p.pose.position.y=y
            is_last = (i == len(pts)-1)
            if is_last and final_yaw is not None:
                yaw=final_yaw  # the goal's own explicit heading, used as-is
            else:
                if i < len(pts)-1:
                    nx,ny=pts[i+1]
                    yaw=math.atan2(ny-y,nx-x)
                else:
                    yaw=prev_yaw
                alpha=0.3
                yaw=prev_yaw+alpha*math.atan2(math.sin(yaw-prev_yaw),math.cos(yaw-prev_yaw))
            prev_yaw=yaw
            p.pose.orientation.z=math.sin(yaw/2.0)
            p.pose.orientation.w=math.cos(yaw/2.0)
            path.poses.append(p)
        self.pub_path.publish(path)

    def line_points(self,sx,sy,gx,gy,frame_id):
        """
            Builds a smoothed straight-line path between two points.

        Args:
            sx (float): Start X.
            sy (float): Start Y.
            gx (float): Goal X.
            gy (float): Goal Y.
            frame_id (str): Unused; kept for call-site symmetry with publish_path.

        Returns:
            list[tuple[float, float]]: Smoothed points from start to goal.
        """
        pts=[]
        for i in range(50+1):
            a=i/50.0
            x=(1-a)*sx+a*gx; y=(1-a)*sy+a*gy
            pts.append((x,y))
        return _catmull_rom_centripetal(pts,8,False)

    def inflate_occupancy(self,occ,w,h,r_cells,occ_th):
        """
            Expands every occupied cell by r_cells in all directions, marking nearby free cells
            as occupied (inflated) without touching cells that are already hard obstacles or
            unknown.

        Args:
            occ (list[int]): Flat raw occupancy grid.
            w (int): Grid width, in cells.
            h (int): Grid height, in cells.
            r_cells (int): Inflation radius, in cells.
            occ_th (int): Occupancy value at/above which a cell counts as an obstacle.

        Returns:
            list[int]: Flat inflated occupancy grid.
        """
        if r_cells<=0: return occ[:]
        inflated = list(occ)
        occ_cells=[(i%w,i//w) for i,v in enumerate(occ) if v>=occ_th and v!=255]
        for ox,oy in occ_cells:
            xmin=max(0,ox-r_cells); xmax=min(w-1,ox+r_cells)
            ymin=max(0,oy-r_cells); ymax=min(h-1,oy+r_cells)
            r2=r_cells*r_cells
            for y in range(ymin,ymax+1):
                dy=y-oy; dy2=dy*dy
                base=y*w
                for x in range(xmin,xmax+1):
                    dx=x-ox
                    if dx*dx+dy2<=r2:
                        idx = base+x
                        if occ[idx] < occ_th and occ[idx] != 255:
                            inflated[idx] = max(inflated[idx], 50)
        return inflated

    def simplify_spacing(self,pts,min_d):
        """
            Drops points closer than min_d to the last kept point, always keeping the first and
            last points.

        Args:
            pts (list[tuple[float, float]]): Points, in order.
            min_d (float): Minimum spacing to keep a point.

        Returns:
            list[tuple[float, float]]: Simplified points.
        """
        if not pts: return pts
        out=[pts[0]]
        for p in pts[1:]:
            if _dist(out[-1],p)>=min_d: out.append(p)
        if out[-1]!=pts[-1]: out.append(pts[-1])
        return out

    def shortcut_path(self,pts):
        """
            Greedily replaces runs of points with a single straight segment wherever the direct
            line between two points is unobstructed.

        Args:
            pts (list[tuple[float, float]]): Points, in order.

        Returns:
            list[tuple[float, float]]: Shortcut points.
        """
        if len(pts)<=2: return pts
        grid_pts=[self.world_to_grid(x,y) for x,y in pts]
        out=[pts[0]]; i=0
        while i<len(grid_pts)-1:
            j=len(grid_pts)-1
            while j>i+1 and not self.grid_line_clear(grid_pts[i],grid_pts[j]): j-=1
            out.append(pts[j]); i=j
        return out

    def grid_line_clear(self,a,b):
        """
            Checks whether a straight line between two grid cells is free of obstacles, walking
            it cell-by-cell via Bresenham's algorithm.

        Args:
            a (tuple[int, int]): Start cell.
            b (tuple[int, int]): End cell.

        Returns:
            bool: True if every cell on the line is in bounds and unoccupied.
        """
        x0,y0=a; x1,y1=b
        dx=abs(x1-x0); dy=abs(y1-y0)
        sx=1 if x0<x1 else -1
        sy=1 if y0<y1 else -1
        err=dx-dy; x,y=x0,y0
        while True:
            if not self.in_bounds(x,y) or self.is_occ(x,y): return False
            if x==x1 and y==y1: break
            e2=2*err
            if e2>-dy: err-=dy; x+=sx
            if e2<dx: err+=dx; y+=sy
        return True

def main(args=None):
    """
        Starts the dijkstra_planner ROS2 node and spins it until shutdown.

    Args:
        args: Command-line arguments passed through to rclpy.init().
    """
    rclpy.init(args=args)
    node=DijkstraPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
