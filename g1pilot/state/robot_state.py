#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from astroviz_interfaces.msg import MotorState, MotorStateList

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


_joint_index_to_ros_name = {
    G1JointIndex.LeftHipPitch: "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll: "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw: "left_hip_yaw_joint",
    G1JointIndex.LeftKnee: "left_knee_joint",
    G1JointIndex.LeftAnklePitch: "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll: "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch: "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll: "right_hip_roll_joint",
    G1JointIndex.RightHipYaw: "right_hip_yaw_joint",
    G1JointIndex.RightKnee: "right_knee_joint",
    G1JointIndex.RightAnklePitch: "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll: "right_ankle_roll_joint",
    G1JointIndex.WaistYaw: "waist_yaw_joint",
    G1JointIndex.WaistRoll: "waist_roll_joint",
    G1JointIndex.WaistPitch: "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch: "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll: "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw: "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow: "left_elbow_joint",
    G1JointIndex.LeftWristRoll: "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch: "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw: "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch: "right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll: "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw: "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow: "right_elbow_joint",
    G1JointIndex.RightWristRoll: "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch: "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw: "right_wrist_yaw_joint",
}


class RobotState(Node):
    def __init__(self):
        super().__init__('robot_state')

        self.declare_parameter('use_robot', True)
        self.declare_parameter('interface', '')
        self.declare_parameter('publish_joint_states', True)

        use_robot_val = self.get_parameter('use_robot').value
        self.use_robot = use_robot_val if isinstance(use_robot_val, bool) else (str(use_robot_val).lower() == 'true')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        pub_js_val = self.get_parameter('publish_joint_states').value
        self.publish_joint_states = pub_js_val if isinstance(pub_js_val, bool) else (str(pub_js_val).lower() == 'true')
        self.ns = '/g1pilot'

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos_profile)
        self.imu_pub = self.create_publisher(Imu, f"{self.ns}/imu", qos_profile)
        self.motor_state_pub = self.create_publisher(MotorStateList, f"{self.ns}/motor_state", qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_indices = sorted(_joint_index_to_ros_name.keys())
        self.joint_names = [_joint_index_to_ros_name[i] for i in self.joint_indices]
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names

        if self.use_robot:
            ChannelFactoryInitialize(0, interface)
            self.subscriber_low_state = ChannelSubscriber("rt/lowstate", LowState_)
            self.subscriber_low_state.Init(self.callback_lowstate)
        else:
            self.create_timer(0.05, self._sim_tick)

    def callback_lowstate(self, msg: LowState_):
        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "pelvis"
        imu_msg.orientation.w = float(msg.imu_state.quaternion[0])
        imu_msg.orientation.x = float(msg.imu_state.quaternion[1])
        imu_msg.orientation.y = float(msg.imu_state.quaternion[2])
        imu_msg.orientation.z = float(msg.imu_state.quaternion[3])
        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu_msg)

        # TF pelvis -> imu_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "pelvis"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = imu_msg.orientation
        self.tf_broadcaster.sendTransform(t)

        # Motor states
        positions = []
        motor_list_msg = MotorStateList()
        for idx in self.joint_indices:
            if idx < len(msg.motor_state):
                m = msg.motor_state[idx]
                motor_state = MotorState()
                motor_state.name = _joint_index_to_ros_name[idx]
                motor_state.temperature = float(m.temperature[0] if hasattr(m.temperature, "__len__") else m.temperature)
                motor_state.voltage = float(m.vol)
                motor_state.position = float(m.q)
                motor_state.velocity = float(m.dq)
                motor_list_msg.motor_list.append(motor_state)
                positions.append(float(m.q))

        self.motor_state_pub.publish(motor_list_msg)

        if self.publish_joint_states:
            self.joint_state_msg.header.stamp = now
            self.joint_state_msg.position = positions
            self.joint_pub.publish(self.joint_state_msg)

    def _sim_tick(self):
        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "pelvis"
        imu_msg.orientation.w = 1.0
        self.imu_pub.publish(imu_msg)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = now
            js.name = self.joint_names
            js.position = [0.0] * len(js.name)
            self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RobotState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
