#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool

def yaw_from_quat(x, y, z, w):
    """
        Extracts the yaw (rotation about Z) from a quaternion.

    Args:
        x (float): Quaternion x component.
        y (float): Quaternion y component.
        z (float): Quaternion z component.
        w (float): Quaternion w component.

    Returns:
        float: Yaw, in radians.
    """
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def wrap_angle(a):
    """
        Wraps an angle to (-pi, pi].

    Args:
        a (float): Angle, in radians.

    Returns:
        float: Equivalent angle in (-pi, pi].
    """
    return math.atan2(math.sin(a), math.cos(a))

class Nav2Point(Node):
    def __init__(self):
        """
            Declares all control parameters, sets up subscriptions/publishers/timer, and
            initializes the controller's pose/path/PID state.
        """
        super().__init__('nav2point')
        self.declare_parameter('publish_rate', 50.0)          # control loop rate, Hz
        self.declare_parameter('pos_kp', 0.8)                 # position PID: proportional gain
        self.declare_parameter('yaw_kp', 1.5)                 # heading PID: proportional gain
        self.declare_parameter('waypoint_tolerance', 0.20)    # distance to advance to next waypoint
        self.declare_parameter('goal_tolerance', 0.10)        # distance to consider the goal reached
        self.declare_parameter('frame_id', 'map')             # default path frame
        self.declare_parameter('joy_topic', '/g1pilot/auto_joy')
        self.declare_parameter('path_topic', '/g1pilot/path')
        self.declare_parameter('auto_enable_topic', '/g1pilot/auto_enable')
        self.declare_parameter('vx_limit', 0.6)   # max forward speed, m/s
        self.declare_parameter('vy_limit', 0.6)   # max lateral speed, m/s (<=0 disables strafing)
        self.declare_parameter('wz_limit', 0.5)   # max yaw rate, rad/s
        self.declare_parameter('align_enter_deg', 20.0)
        self.declare_parameter('align_exit_deg', 8.0)
        self.declare_parameter('pos_ki', 0.05)                # position PID: integral gain
        self.declare_parameter('pos_kd', 0.15)                # position PID: derivative gain
        self.declare_parameter('pos_integral_limit', 0.3)     # bounds ki's own contribution
        self.declare_parameter('yaw_ki', 0.1)                 # heading PID: integral gain
        self.declare_parameter('yaw_kd', 0.1)                 # heading PID: derivative gain
        self.declare_parameter('yaw_integral_limit', 0.3)
        self.declare_parameter('final_wz_limit', 0.2)
        self.declare_parameter('final_yaw_kp', 0.8)
        self.declare_parameter('final_yaw_ki', 0.05)
        self.declare_parameter('final_yaw_kd', 0.05)
        self.declare_parameter('final_yaw_integral_limit', 0.3)

        self.rate = self.get_parameter('publish_rate').value
        self.pos_kp = self.get_parameter('pos_kp').value
        self.pos_ki = self.get_parameter('pos_ki').value
        self.pos_kd = self.get_parameter('pos_kd').value
        self.pos_integral_limit = self.get_parameter('pos_integral_limit').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.yaw_ki = self.get_parameter('yaw_ki').value
        self.yaw_kd = self.get_parameter('yaw_kd').value
        self.yaw_integral_limit = self.get_parameter('yaw_integral_limit').value
        self.final_wz_lim = self.get_parameter('final_wz_limit').value
        self.final_yaw_kp = self.get_parameter('final_yaw_kp').value
        self.final_yaw_ki = self.get_parameter('final_yaw_ki').value
        self.final_yaw_kd = self.get_parameter('final_yaw_kd').value
        self.final_yaw_integral_limit = self.get_parameter('final_yaw_integral_limit').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.vx_lim = self.get_parameter('vx_limit').value
        self.vy_lim = self.get_parameter('vy_limit').value
        self.wz_lim = self.get_parameter('wz_limit').value
        self.align_enter = math.radians(self.get_parameter('align_enter_deg').value)
        self.align_exit = math.radians(self.get_parameter('align_exit_deg').value)
        self.dt = 1.0 / self.rate
        self.auto_enable_topic = self.get_parameter('auto_enable_topic').value

        qos = QoSProfile(depth=10)
        self.sub_odom = self.create_subscription(Odometry, '/lidar_odometry/pose_fixed', self.cb_odom, qos)
        self.sub_auto_enable = self.create_subscription(Bool, self.auto_enable_topic, self.cb_auto_enable, qos)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, qos)
        self.pub_joy = self.create_publisher(Joy, self.joy_topic, qos)
        self.pub_wp_marker = self.create_publisher(Marker, '/g1pilot/waypoint_marker', qos)
        self.pub_goal_marker = self.create_publisher(Marker, '/g1pilot/goal_marker', qos)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

        self.have_pose = False
        self.auto_enabled = True
        self.path = []
        self.path_frame = self.frame_id
        self.idx = 0
        self.x = self.y = self.yaw = 0.0
        self.aligning = True
        self.final_yaw = None
        self.finishing_orientation = False
        self.reset_pid_state()

        self.logged_no_pose = False
        self.logged_no_path = False
        self.logged_end_path = False

    def cb_odom(self, msg: Odometry):
        """
            Updates the robot's tracked position and yaw from odometry.

        Args:
            msg (nav_msgs.msg.Odometry): Latest odometry reading.
        """
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self.yaw = yaw_from_quat(qx, qy, qz, qw)
        self.have_pose = True
        self.logged_no_pose = False

    def cb_auto_enable(self, msg: Bool):
        """
            Turns autonomous following on/off.

        Args:
            msg (std_msgs.msg.Bool): True to enable following the current path.
        """
        self.auto_enabled = msg.data

    def cb_path(self, msg: Path):
        """
            Adopts a new path to follow, preserving in-progress final-orientation state if the
            new path targets the same endpoint (see loop()'s stickiness).

        Args:
            msg (nav_msgs.msg.Path): New path; its last pose's orientation is the goal's own
                explicit heading, used once position is reached.
        """
        self.get_logger().info(f"Received new path with {len(msg.poses)} points.")
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        new_final_yaw = None
        if msg.poses:
            q = msg.poses[-1].pose.orientation
            new_final_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        same_goal = (
            self.finishing_orientation and new_path and self.path
            and math.hypot(new_path[-1][0] - self.path[-1][0], new_path[-1][1] - self.path[-1][1]) <= self.goal_tol
        )

        self.path = new_path
        self.final_yaw = new_final_yaw
        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.frame_id
        self.logged_no_path = False
        self.logged_end_path = False
        if not same_goal:
            
            self.idx = 0
            while (self.idx < len(self.path) - 1
                   and math.hypot(self.path[self.idx][0] - self.x, self.path[self.idx][1] - self.y) <= self.wp_tol):
                self.idx += 1
            self.aligning = True
            self.finishing_orientation = False
            self.reset_pid_state()
        if self.path:
            self.publish_goal_marker(self.path[-1][0], self.path[-1][1])

    def reset_pid_state(self):
        """
            Clears integral/derivative memory. Called whenever the tracked error's reference
            changes discretely -- a new path, a new waypoint, entering/leaving the align-in-place
            phase -- so error history against the *previous* target doesn't leak in as a bogus
            integral bias or a derivative "kick" at the discontinuity.
        """
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.prev_dx = None
        self.prev_dy = None
        self.prev_yaw_err = None
        self.final_integral_yaw = 0.0
        self.final_prev_yaw_err = None

    def compute_yaw_wz(self, yaw_err):
        """
            PID on heading error -> clamped wz, for normal waypoint tracking (aligning + driving
            toward the current waypoint). See compute_final_yaw_wz for the separate one used once
            position is reached and only the final orientation is left to fix.
        """
        if self.prev_yaw_err is None:
            self.prev_yaw_err = yaw_err
        self.integral_yaw = max(-self.yaw_integral_limit,
                                 min(self.yaw_integral_limit, self.integral_yaw + yaw_err * self.dt))
        d_yaw = (yaw_err - self.prev_yaw_err) / self.dt
        self.prev_yaw_err = yaw_err
        raw_wz = self.yaw_kp * yaw_err + self.yaw_ki * self.integral_yaw + self.yaw_kd * d_yaw
        return max(-self.wz_lim, min(self.wz_lim, raw_wz))

    def compute_final_yaw_wz(self, yaw_err):
        """
            PID on heading error -> clamped wz, for the final-orientation fix-up phase only.
            Separate gains, wz cap, and integral/derivative state from compute_yaw_wz -- see the
            final_wz_limit/final_yaw_* parameter comments for why this phase wants to turn slower
            and gentler rather than reusing the waypoint-tracking gains.
        """
        if self.final_prev_yaw_err is None:
            self.final_prev_yaw_err = yaw_err
        self.final_integral_yaw = max(-self.final_yaw_integral_limit,
                                       min(self.final_yaw_integral_limit, self.final_integral_yaw + yaw_err * self.dt))
        d_yaw = (yaw_err - self.final_prev_yaw_err) / self.dt
        self.final_prev_yaw_err = yaw_err
        raw_wz = self.final_yaw_kp * yaw_err + self.final_yaw_ki * self.final_integral_yaw + self.final_yaw_kd * d_yaw
        return max(-self.final_wz_lim, min(self.final_wz_lim, raw_wz))

    def publish_goal_marker(self, gx, gy):
        """
            Publishes an RViz marker at the final goal position.

        Args:
            gx (float): Goal X.
            gy (float): Goal Y.
        """
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_goal'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
        self.pub_goal_marker.publish(m)

    def publish_wp_marker(self, wx, wy):
        """
            Publishes an RViz marker at the current waypoint.

        Args:
            wx (float): Waypoint X.
            wy (float): Waypoint Y.
        """
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_wp'
        m.id = 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
        self.pub_wp_marker.publish(m)

    def publish_stop(self):
        """
            Publishes an all-zero Joy. Called every tick (not just once) whenever there's no
            active path to follow, so a lost/dropped single stop message can never leave the robot
            stuck holding its last nonzero command. buttons[7] must be 1 (the "deadman" held) --
            loco_client's joystick_callback only reacts to axes/buttons at all when it's set; with it
            at 0 (as a truly empty Joy would have it), the whole is_stop branch that actually
            publishes the explicit zero command to /run_command/cmd never runs, and the sim bridge
            just keeps executing whatever velocity it last received.
        """
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 14
        joy.buttons[7] = 1
        self.pub_joy.publish(joy)

    def loop(self):
        """
            Runs at publish_rate: follows the current path (align-then-drive per waypoint),
            then rotates in place to the goal's final heading once position is reached, publishing
            a stop whenever there's no path/pose or auto-following is disabled.
        """
        try:
            if not self.auto_enabled:
                return

            if (len(self.path) == 0):
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                self.publish_stop()
                return

            if not self.have_pose and self.auto_enabled:
                if not self.logged_no_pose:
                    self.get_logger().warn('No pose available.')
                    self.logged_no_pose = True
                self.publish_stop()
                return

            if (not self.path or len(self.path) == 0) and self.auto_enabled:
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                self.publish_stop()
                return

            self.logged_no_pose = False
            self.logged_no_path = False
            self.logged_end_path = False

            dist_goal = math.hypot(self.path[-1][0] - self.x, self.path[-1][1] - self.y)

            if self.finishing_orientation or dist_goal <= self.goal_tol:
                if self.final_yaw is None:
                    self.path = []
                    self.finishing_orientation = False
                    self.publish_stop()
                    return
                if not self.finishing_orientation:
                    self.finishing_orientation = True
                    self.reset_pid_state()
                    self.get_logger().info(
                        f"Position reached. Fixing final orientation: current={math.degrees(self.yaw):.1f} deg, "
                        f"target={math.degrees(self.final_yaw):.1f} deg."
                    )
                final_yaw_err = wrap_angle(self.final_yaw - self.yaw)
                if abs(final_yaw_err) <= self.align_exit:
                    self.finishing_orientation = False
                    self.path = []
                    self.get_logger().info(f"Final orientation reached: {math.degrees(self.yaw):.1f} deg.")
                    self.publish_stop()
                    return
                self.publish_joy_cmd(0.0, 0.0, self.compute_final_yaw_wz(final_yaw_err))
                return

            if self.idx >= len(self.path) and self.auto_enabled:
                if not self.logged_end_path:
                    self.get_logger().warn('Reached the end of the path.')
                    self.logged_end_path = True
                self.publish_stop()
                return

            wx, wy = self.path[self.idx]
            dx = wx - self.x
            dy = wy - self.y
            dist_wp = math.hypot(dx, dy)

            if self.idx < len(self.path) - 1 and dist_wp <= self.wp_tol:
                self.idx += 1
                self.aligning = True  # face the new waypoint before moving toward it
                self.reset_pid_state()
                wx, wy = self.path[self.idx]
                dx = wx - self.x
                dy = wy - self.y
                dist_wp = math.hypot(dx, dy)

            self.publish_wp_marker(wx, wy)

            desired_yaw = math.atan2(dy, dx)
            yaw_err = wrap_angle(desired_yaw - self.yaw)

            if self.aligning:
                if abs(yaw_err) <= self.align_exit:
                    self.aligning = False
                    self.reset_pid_state()
            elif abs(yaw_err) >= self.align_enter:
                self.aligning = True
                self.reset_pid_state()

            if self.vy_lim <= 0.0:
                
                if self.aligning:
                    vx_b = 0.0
                else:
                    dist_to_wp = math.hypot(dx, dy)
                    if self.prev_dx is None:
                        self.prev_dx = dist_to_wp
                    self.integral_x = max(-self.pos_integral_limit,
                                           min(self.pos_integral_limit, self.integral_x + dist_to_wp * self.dt))
                    d_dist = (dist_to_wp - self.prev_dx) / self.dt
                    self.prev_dx = dist_to_wp

                    raw_vx = self.pos_kp * dist_to_wp + self.pos_ki * self.integral_x + self.pos_kd * d_dist
                    vx_b = max(0.0, min(self.vx_lim, raw_vx))  # never negative -- no reverse
                vy_b = 0.0
            elif self.aligning:
                vx_b = vy_b = 0.0
            else:
                if self.prev_dx is None:
                    self.prev_dx, self.prev_dy = dx, dy
                self.integral_x = max(-self.pos_integral_limit,
                                       min(self.pos_integral_limit, self.integral_x + dx * self.dt))
                self.integral_y = max(-self.pos_integral_limit,
                                       min(self.pos_integral_limit, self.integral_y + dy * self.dt))
                d_dx = (dx - self.prev_dx) / self.dt
                d_dy = (dy - self.prev_dy) / self.dt
                self.prev_dx, self.prev_dy = dx, dy

                raw_vx_w = self.pos_kp * dx + self.pos_ki * self.integral_x + self.pos_kd * d_dx
                raw_vy_w = self.pos_kp * dy + self.pos_ki * self.integral_y + self.pos_kd * d_dy
                vx_w = max(-self.vx_lim, min(self.vx_lim, raw_vx_w))
                vy_w = max(-self.vy_lim, min(self.vy_lim, raw_vy_w))

                c = math.cos(-self.yaw)
                s = math.sin(-self.yaw)
                vx_b = c * vx_w - s * vy_w
                vy_b = s * vx_w + c * vy_w

            self.publish_joy_cmd(vx_b, vy_b, self.compute_yaw_wz(yaw_err))

        except Exception as e:
            self.get_logger().error(f'Error in loop: {e}')

    def publish_joy_cmd(self, vx_b, vy_b, wz):
        """
            Publishes a body-frame velocity command as a Joy message.

        Args:
            vx_b (float): Forward speed, m/s (already clamped to vx_limit).
            vy_b (float): Lateral speed, m/s (already clamped to vy_limit).
            wz (float): Yaw rate, rad/s (already clamped to wz_limit).
        """
        
        axes = [0.0] * 8
        buttons = [0] * 14
        axes[1] = -vx_b
        axes[0] = -vy_b
        axes[2] = -wz
        buttons[7] = 1

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = axes
        joy.buttons = buttons
        self.pub_joy.publish(joy)

def main(args=None):
    """
        Starts the nav2point ROS2 node and spins it until shutdown.

    Args:
        args: Command-line arguments passed through to rclpy.init().
    """
    rclpy.init(args=args)
    node = Nav2Point()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
