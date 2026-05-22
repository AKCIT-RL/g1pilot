#!/usr/bin/env python3
import subprocess
import threading
import time
import numpy as np
import array

from tf2_ros import TransformBroadcaster
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyaffine3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped,Point
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState

from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from scipy.spatial.transform import Rotation as R

import pyopensot as pysot
from pyopensot.tasks.velocity import Postural, Cartesian, CoM
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
from pyopensot_collision.constraints.velocity import CollisionAvoidance

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointWaistIndex,
    G1_29_JointIndex,
    DataBuffer,
)

G1_NUM_MOTOR = 29 # 12 body + 17 arm

q_init = [
        -0.1,
    0.0,
    0.0,  # hips
    0.432,  # knee
    -0.317,
    0.0,  # ankles
    -0.1,
    0.0,
    0.0,  # hips
    0.432,  # knee
    -0.317,
    0.0,  # ankles
    0.0,
    0.0,
    0.0,  # waist
    0.3,
    0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,  # arm
    0.3,
    -0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,
]  # arm


class Mode:
    PR = 0
    AB = 1


class G1CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("g1_collision_avoidance_node")
        self.get_logger().info("Starting G1 Collision Avoidance Node")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("enable_collision_avoidance", False)
        self.declare_parameter("interface", "")
        self.declare_parameter("send_cmds_to_robot", True)
        self.declare_parameter("publish_joint_states_opensot", False)
        self.interface = str(self.get_parameter("interface").value)
        use_robot_val = self.get_parameter("use_robot").value
        self.use_robot = use_robot_val if isinstance(use_robot_val, bool) else (str(use_robot_val).lower() == 'true')
        self.enable_collision_avoidance = bool(self.get_parameter("enable_collision_avoidance").value)
        self.send_cmds_to_robot = bool(self.get_parameter("send_cmds_to_robot").value)
        pub_js_val = self.get_parameter("publish_joint_states_opensot").value
        self.publish_joint_states_opensot = pub_js_val if isinstance(pub_js_val, bool) else (str(pub_js_val).lower() == 'true')

        self.control_dt = 0.005
        self.time = 0.0
        self.t = 0.0
        self.init_duration_s = 3.0

        self.mode = Mode.PR
        self.mode_machine = 0
        self.motors_on = 1

        self.right_hand_pose_ref = None
        self.left_hand_pose_ref = None
        self.emergency_stop = False
        self._initialized = False
        self.start_opensot = False



        self.client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.base_height_publisher = self.create_publisher(Float64, "/base_height", 10)
        self.base_link_broadcaster = TransformBroadcaster(self)

        # statis transform between world and pelvis
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.base_link_broadcaster.sendTransform(t)

        self.start_opensot_sub = self.create_subscription(Bool, "/g1pilot/start_opensot", self.start_opensot_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, "/g1pilot/emergency_stop", self.emergency_stop_callback, 10)
        self.righ_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/right_hand/pose_ref", self.right_hand_pose_ref_callback, 10
        )
        self.left_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/left_hand/pose_ref", self.left_hand_pose_ref_callback, 10
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service /robot_state_publisher/get_parameters not available, waiting...")

        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.urdf = None
        if future.result() is not None:
            values = future.result().values
            for val in values:
                self.urdf = val.string_value
        else:
            self.get_logger().error("Failed to get robot_description from parameter server")

        self.interactive_marker_server = InteractiveMarkerServer(self, "teleoperation_markers")
        self.marker_poses = {}
        self.marker_enabled = {}
        self.menu_handler = {}
        self.menu_entry_ids = {}

        self.collision_distances_publisher = self.create_publisher(Marker, 'collision_distances', 10)

        self.right_hand_frame_ref = "pelvis"
        self.left_hand_frame_ref = "pelvis"

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()

        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.subscribe_thread = None
        self.crc = None
        self.msg = None
        self.all_motor_q = None

        if self.use_robot:
            self.get_logger().info("use_robot=True -> Initializing Unitree DDS interface")
            self.initialize_interface()
        else:
            self.get_logger().warn("use_robot=False -> Running in simulation/visualization mode (publishing only /joint_states)")

        self.initialize()
        self.initialize_imarkers()

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

    def _subscribe_motor_state(self):
        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def initialize_interface(self):
        ChannelFactoryInitialize(0, self.interface)

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()

        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 150.0
        self.kd_low = 4.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self._initialized = True

    def get_mode_machine(self) -> int:
        msg = self.lowstate_buffer.GetData()
        return msg.mode_machine

    def get_current_motor_q(self):
        msg = self.lowstate_buffer.GetData()
        if msg is None:
            return np.zeros(29)
        q = np.zeros(29)
        for i in range(29):
            q[i] = msg.motor_state[i].q
        return q

    def right_hand_pose_ref_callback(self, msg: PoseStamped):
        self.right_hand_pose_ref = msg

    def left_hand_pose_ref_callback(self, msg: PoseStamped):
        self.left_hand_pose_ref = msg

    def start_opensot_callback(self, msg: Bool):
        self.start_opensot = bool(msg.data)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)

    def initialize_imarkers(self):
        self.marker_enabled = {}
        self.menu_handler = {}

        base_ref, _ = self.base.getReference()
        com_ref, _ = self.com.getReference()
        pose_ref = pyaffine3.Affine3()
        pose_ref.translation = com_ref
        pose_ref.linear = base_ref.linear.copy()
        # self.get_logger().info(f"Initial base pose:\n{pose_ref}")
        # self.make_6dof_marker("base_marker", pose_ref, "world")

        right_hand_ref = self.right_gripper.getReference()
        self.get_logger().info(f"Initial right hand pose:\n{right_hand_ref}")
        self.make_6dof_marker("right_hand_marker", right_hand_ref[0], self.right_hand_frame_ref)

        left_hand_ref = self.left_gripper.getReference()
        self.get_logger().info(f"Initial left hand pose:\n{left_hand_ref}")
        self.make_6dof_marker("left_hand_marker", left_hand_ref[0], self.left_hand_frame_ref)

    def make_6dof_marker(self, name, pose, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = '6-DOF Control'
        int_marker.scale = 0.3

        int_marker.pose.position.x = pose.translation[0]
        int_marker.pose.position.y = pose.translation[1]
        int_marker.pose.position.z = pose.translation[2]

        quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
        int_marker.pose.orientation.x = quat_xyzw[0]
        int_marker.pose.orientation.y = quat_xyzw[1]
        int_marker.pose.orientation.z = quat_xyzw[2]
        int_marker.pose.orientation.w = quat_xyzw[3]

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose = int_marker.pose
        self.marker_poses[name] = ps

        self.marker_home_poses = getattr(self, "marker_home_poses", {})
        self.marker_home_poses[name] = PoseStamped()
        self.marker_home_poses[name].header.frame_id = frame_id
        self.marker_home_poses[name].pose = int_marker.pose

        # Add a visible marker (e.g., a cube)
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        int_marker.controls.append(control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)

        self.marker_enabled[name] = True

        menu = MenuHandler()
        h_enable = menu.insert("Enable", callback=self.process_menu)
        menu.setCheckState(
            h_enable,
            MenuHandler.CHECKED if self.marker_enabled.get(name, True) else MenuHandler.UNCHECKED
        )
        h_reset = menu.insert("Reset", callback=self.process_menu)
        self.menu_handler[name] = menu

        self.menu_entry_ids = getattr(self, "menu_entry_ids", {})
        self.menu_entry_ids[name] = {"enable": h_enable, "reset": h_reset}

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.interactive_marker_server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        menu.apply(self.interactive_marker_server, name)
        self.interactive_marker_server.applyChanges()

        menu.setCheckState(
            h_enable,
            MenuHandler.CHECKED if self.marker_enabled.get(name, True) else MenuHandler.UNCHECKED
        )

        self.menu_handler[name] = menu

        self.menu_entry_ids = getattr(self, "menu_entry_ids", {})
        self.menu_entry_ids[name] = {"enable": h_enable, "reset": h_reset}

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.interactive_marker_server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        menu.apply(self.interactive_marker_server, name)
        self.interactive_marker_server.applyChanges()

    def process_feedback(self, feedback):
        name = feedback.marker_name
        if not self.marker_enabled.get(name, True):
            return

        if name not in self.marker_poses:
            self.marker_poses[name] = PoseStamped()

        self.marker_poses[name].header = feedback.header
        self.marker_poses[name].pose = feedback.pose

    def process_menu(self, feedback):
        name = feedback.marker_name
        ids = self.menu_entry_ids.get(name, {})

        if feedback.menu_entry_id == ids.get("enable"):
            new_state = not self.marker_enabled.get(name, True)
            self.marker_enabled[name] = new_state

            menu = self.menu_handler.get(name, None)
            if menu is not None:
                menu.setCheckState(
                    ids["enable"],
                    MenuHandler.CHECKED if new_state else MenuHandler.UNCHECKED
                )
                menu.reApply(self.interactive_marker_server)
                self.interactive_marker_server.applyChanges()


        elif feedback.menu_entry_id == ids.get("reset"):
            home = self.marker_home_poses.get(name, None)
            if home is not None:
                self.marker_poses[name] = PoseStamped()
                self.marker_poses[name].header = home.header
                self.marker_poses[name].pose = home.pose

                self.interactive_marker_server.setPose(name, home.pose, home.header)
                self.interactive_marker_server.applyChanges()


    def add_6dof_controls(self, marker):
        axes = ['x', 'y', 'z']
        for axis in axes:
            # Rotation
            control = InteractiveMarkerControl()
            control.name = f'rotate_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

            # Translation
            control = InteractiveMarkerControl()
            control.name = f'move_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

    # ----------------------------
    # OpenSoT init
    # ----------------------------
    def initialize(self, manipulation_frame="world"):
        self.get_logger().warning("Initializing XBot2 Model Interface")

        if not self.urdf or len(self.urdf.strip()) < 100:
            self.get_logger().error(f"robot_description invalid. len={0 if not self.urdf else len(self.urdf)}")
            raise RuntimeError("robot_description is empty/invalid -> cannot build ModelInterface2")


        self.model = xbi.ModelInterface2(self.urdf)
        
        self.q = np.zeros(self.model.nq)
        self.q[2] = 0.6756
        self.q[6] = 1.0
        self.q[7: ] = q_init[0:].copy()

        self.dq = np.zeros(self.model.nv)

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        self.com = CoM(self.model)

        self.get_logger().warning("Initializing OpenSoT Tasks and Constraints")

        self.get_logger().info("Task: Base")
        self.base = Cartesian("base_task", self.model, "pelvis", manipulation_frame)
        self.base.setLambda(0.1)

        self.get_logger().info("Task: Torso")
        self.torso = Cartesian("torso_task", self.model, "torso_link", manipulation_frame)
        self.torso.setLambda(0.1)

        self.get_logger().info("Task: Right Gripper")
        self.right_gripper = Cartesian(
            "right_gripper_task",
            self.model,
            "right_hand_point_contact",
            self.right_hand_frame_ref,
        )
        self.right_gripper.setLambda(0.1)

        self.get_logger().info("Task: Left Gripper")
        self.left_gripper = Cartesian(
            "left_gripper_task",
            self.model,
            "left_hand_point_contact",
            self.left_hand_frame_ref,
        )
        self.left_gripper.setLambda(0.1)

        self.get_logger().info("Task: Postural")
        self.postural = Postural(self.model)
        self.postural.setLambda(0.1)
        self.W = self.postural.getWeight().copy()
        #W[0:6, 0:6] = 0.0
        #W[6:10, 6:10] = 0.0
        print(self.W.shape)
        self.postural.setWeight(self.W)

        print(self.model.getNv())
        print(self.model.getNq())

        self.get_logger().info("Constraints: Joint Limits")
        self.qmin, self.qmax = self.model.getJointLimits()
        self.qlims = JointLimits(self.model, self.qmax, self.qmin)
        print(self.qmin)
        self.dqmax = self.model.getVelocityLimits()
        self.dqlims = VelocityLimits(self.model, self.dqmax, self.control_dt)

        # self.collision_avoidance_constraint = None
        # if self.enable_collision_avoidance:
        self.get_logger().info("Constraints: Self-Collision Avoidance")
        self.collision_avoidance_constraint = CollisionAvoidance(
            self.model, max_pairs=50, collision_urdf=self.urdf)#, collision_srdf=self.urdf)

        # All arm links and torso now have primitive collision geometries in g1_29dof.urdf.
        collision_list = {
            # Left arm vs torso
            ("left_shoulder_yaw_link", "torso_link"),
            ("left_elbow_link", "torso_link"),
            ("left_wrist_roll_link", "torso_link"),
            ("left_wrist_pitch_link", "torso_link"),
            ("left_wrist_yaw_link", "torso_link"),
            ("left_rubber_hand", "torso_link"),
            # Right arm vs torso
            ("right_shoulder_yaw_link", "torso_link"),
            ("right_elbow_link", "torso_link"),
            ("right_wrist_roll_link", "torso_link"),
            ("right_wrist_pitch_link", "torso_link"),
            ("right_wrist_yaw_link", "torso_link"),
            ("right_rubber_hand", "torso_link"),
            # hip
            ("left_rubber_hand", "waist_yaw_link"),
            ("right_rubber_hand", "waist_yaw_link"),
            # pelvis
            ("left_rubber_hand", "pelvis_contour_link"),
            ("right_rubber_hand", "pelvis_contour_link"),
            # Left hand vs legs
            ("left_rubber_hand", "left_hip_pitch_link"),
            ("left_rubber_hand", "left_hip_roll_link"),
            ("left_rubber_hand", "left_hip_yaw_link"),
            ("left_rubber_hand", "left_knee_link"),
            ("left_rubber_hand", "right_hip_pitch_link"),
            ("left_rubber_hand", "right_hip_roll_link"),
            ("left_rubber_hand", "right_hip_yaw_link"),
            ("left_rubber_hand", "right_knee_link"),
            # Right hand vs legs
            ("right_rubber_hand", "left_hip_pitch_link"),
            ("right_rubber_hand", "left_hip_roll_link"),
            ("right_rubber_hand", "left_hip_yaw_link"),
            ("right_rubber_hand", "left_knee_link"),
            ("right_rubber_hand", "right_hip_pitch_link"),
            ("right_rubber_hand", "right_hip_roll_link"),
            ("right_rubber_hand", "right_hip_yaw_link"),
            ("right_rubber_hand", "right_knee_link"),
        }

        self.collision_avoidance_constraint.setCollisionList(collision_list)
        self.collision_avoidance_constraint.setBoundScaling(0.1)
        self.collision_avoidance_constraint.setLinkPairThreshold(0.01)
        self.collision_avoidance_constraint.setDetectionThreshold(-1)
        

        # self.com_xy = self.com % [0, 1]
        # self.stack = (
        #     self.com_xy
        #     / (self.base % [3, 4, 5] + self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
        #     / self.postural
        #     << self.qlims
        #     << self.dqlims
        # )
        self.stack = ((
            self.base#%[0,1,3,4,5]
            / (self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
            / self.postural)
            << self.qlims
            << self.dqlims
            << self.collision_avoidance_constraint
        )
            
        self.stack.update()
        self.solver = pysot.iHQP(self.stack, eps_regularisation=1e11)

    # ----------------------------
    # Control loop
    # ----------------------------
    def control_loop(self):
        wrist_vals = {m.value for m in G1_29_JointWristIndex}

        if self.start_opensot and not self.emergency_stop:
            self.model.setJointPosition(self.q)
            self.model.setJointVelocity(self.dq)
            self.model.update()

            # right hand
            use_marker_right = self.marker_enabled.get("right_hand_marker", False) and "right_hand_marker" in self.marker_poses
            if use_marker_right:
                ps = self.marker_poses["right_hand_marker"]
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.right_gripper.setReference(T)
            elif self.right_hand_pose_ref is not None:
                ps = self.right_hand_pose_ref
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.right_gripper.setReference(T)

            # left hand
            use_marker_left = self.marker_enabled.get("left_hand_marker", False) and "left_hand_marker" in self.marker_poses
            if use_marker_left:
                ps = self.marker_poses["left_hand_marker"]
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.left_gripper.setReference(T)
            elif self.left_hand_pose_ref is not None:
                ps = self.left_hand_pose_ref
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.left_gripper.setReference(T)

            # solve
            self.stack.update()
            try:
                dq = self.solver.solve()
                self.q = self.model.sum(self.q, dq)
                self.dq = dq

            except Exception as e:
                self.get_logger().error(f"OpenSoT Solver Error: {e}")
                dq = None

        

            t = TransformStamped()
            t.header.frame_id = "world"
            t.child_frame_id = "pelvis"
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x = self.q[0]
            t.transform.translation.y = self.q[1]
            t.transform.translation.z = self.q[2]
            t.transform.rotation.x = self.q[3]
            t.transform.rotation.y = self.q[4]
            t.transform.rotation.z = self.q[5]
            t.transform.rotation.w = self.q[6]

            self.base_link_broadcaster.sendTransform(t)


            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()


            try:
                js.name = self.model.getJointNames()[1::]
                js.position = self.q[7:].tolist()
            except Exception:
                js.name = []
                js.position = []
                self.get_logger().error("Error getting joint names from model")
            if self.publish_joint_states_opensot:
                self.joint_state_publisher.publish(js)

            msg = Float64()
            msg.data = self.q[2]

            self.base_height_publisher.publish(msg)

            if not self.use_robot:
                return

            if self.lowcmd_publisher is None or self.msg is None or self.crc is None:
                return

            if self.emergency_stop or not self.motors_on:
                for jid in G1_29_JointArmIndex:
                    self.msg.motor_cmd[jid].mode = 0
                    if jid.value in wrist_vals:
                        self.msg.motor_cmd[jid].kp = self.kp_wrist
                        self.msg.motor_cmd[jid].kd = self.kd_wrist
                    else:
                        self.msg.motor_cmd[jid].kp = self.kp_low
                        self.msg.motor_cmd[jid].kd = self.kd_low

                    self.msg.motor_cmd[jid].q = float(self.msg.motor_cmd[jid].q)

                self.msg.crc = self.crc.Crc(self.msg)
                if self.send_cmds_to_robot:
                    self.lowcmd_publisher.Write(self.msg)
                return

            for jid in G1_29_JointArmIndex:
                self.msg.mode_machine = self.get_mode_machine() 
                self.msg.motor_cmd[jid].mode = 1
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low

                self.msg.motor_cmd[jid].q = float(self.q[7 + jid.value])

            for wid in G1_29_JointWaistIndex:
                self.msg.motor_cmd[wid].mode = 1
                self.msg.motor_cmd[wid].kp = self.kp_low 
                self.msg.motor_cmd[wid].kd = self.kd_low
                self.msg.motor_cmd[wid].dq = 0.0
                self.msg.motor_cmd[wid].tau = 0.0

                self.msg.motor_cmd[wid].q = float(self.q[7 + wid.value])

            try:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            except Exception:
                pass

            self.msg.mode_pr = 1
            self.msg.crc = self.crc.Crc(self.msg)

            if self.send_cmds_to_robot:
                self.lowcmd_publisher.Write(self.msg)

            # publish self-collision debugging
            self.publishCollisionDistances(self.collision_avoidance_constraint.getOrderedWitnessPointVector(), self.get_clock().now().to_msg())

    def publishCollisionDistances(self, collision_distance_points, time):
        marker = Marker()
        marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        for point_pairs in collision_distance_points:
            pa = point_pairs[0]
            pb = point_pairs[1]

            point_a = Point()
            point_a.x = pa[0]
            point_a.y = pa[1]
            point_a.z = pa[2]

            point_b = Point()
            point_b.x = pb[0]
            point_b.y = pb[1]
            point_b.z = pb[2]

            marker.points.append(point_a)
            marker.points.append(point_b)


        self.collision_distances_publisher.publish(marker)
        


def main(args=None):
    rclpy.init(args=args)
    node = G1CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
