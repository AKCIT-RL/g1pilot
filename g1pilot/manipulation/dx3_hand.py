#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from astroviz_interfaces.msg import MotorState, MotorStateList

#CLOSE_RIGHT_VALUES = [-0.10, 0.63, -1.74, 1.06, 0.95, 0.91, 1.22]
CLOSE_RIGHT_VALUES_1 = [0.00,  0.0,  0.0, 1.2, 1.6, 0.0, 0.0] # 1 finger
CLOSE_RIGHT_VALUES_2 = [0.00,  0.0,  0.0, 1.2, 1.6, 1.2, 1.6] # closed Hand
CLOSE_LEFT_VALUES_1  = [0.04,  0.6,  1.4, -1.2, -1.4, -0.0, 0.0] # 1 finger
CLOSE_LEFT_VALUES_2  = [0.04,  0.6,  1.4, -1.2, -1.6, -1.2, -1.4] # closed Hand
# CLOSE_LEFT_VALUES  = [0.04,  -0.04,  1.51, -1.10, -1.47, -1.13, -1.23]
# CLOSE_LEFT_VALUES  = [0.04,  0.4,  1.5, -1.10, -1.58, -1.13, -1.32] motor gripper

OPEN_VALUES          = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class DX3Controller(Node):
    def __init__(self):
        super().__init__('dx3_hand_controller')
        self.declare_parameter("interface", "")
        self.declare_parameter("arm_controlled", "both")
        self.declare_parameter("use_robot", True)
        interface = self.get_parameter("interface").get_parameter_value().string_value
        arm_controlled = self.get_parameter("arm_controlled").get_parameter_value().string_value
        self.use_robot = self.get_parameter("use_robot").get_parameter_value().bool_value

        self.left_gripper_state_publisher = self.create_publisher(MotorStateList, 'g1pilot/dx3/left/motor_state', QoSProfile(depth=10))
        self.right_gripper_state_publisher = self.create_publisher(MotorStateList, 'g1pilot/dx3/right/motor_state', QoSProfile(depth=10))

        self.right_action = None
        self.left_action = None
        self.right_target = OPEN_VALUES
        self.left_target = OPEN_VALUES
        self.total_motors = 7
        self.send_commands = True

        if self.use_robot:
            ChannelFactoryInitialize(0, interface)
        else:
            self.get_logger().info("use_robot:=false -> Not connecting to real hands.")

        if arm_controlled in ["right", "both"]:
            if self.use_robot:
                self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
                self.right_pub.Init()
                self.right_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
                self.right_sub.Init(self.right_callback)
            self.create_subscription(PointStamped, "/g1pilot/right_hand/dx3/action", self.right_action_callback, 10)

        if arm_controlled in ["left", "both"]:
            if self.use_robot:
                self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
                self.left_pub.Init()
                self.left_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
                self.left_sub.Init(self.left_callback)
            self.create_subscription(PointStamped, "/g1pilot/left_hand/dx3/action", self.left_action_callback, 10)

        self.create_timer(0.05, self.publish_commands)

    def right_action_callback(self, msg: PointStamped):
        if msg.point.x < -0.5:
            self.right_action = "close_1"
            self.right_target = CLOSE_RIGHT_VALUES_1
        elif msg.point.x > 0.5:
            self.right_action = "open"
            self.right_target = OPEN_VALUES
        else:
            self.right_action = "close_2"
            self.right_target = CLOSE_RIGHT_VALUES_2

    def left_action_callback(self, msg: PointStamped):
        if msg.point.x < -0.5:
            self.left_action = "close_1"
            self.left_target = CLOSE_LEFT_VALUES_1
        elif msg.point.x > 0.5:
            self.left_action = "open"
            self.left_target = OPEN_VALUES
        else:
            self.left_action = "close_2"
            self.left_target = CLOSE_LEFT_VALUES_2

    def left_callback(self, msg: HandState_):
        motor_list_msg = MotorStateList()
        positions = []
        for i in range(len(msg.motor_state)):
            motor_state = MotorState()
            motor_state.name = f"left_motor_{i}"
            motor_state.temperature = float(msg.motor_state[i].temperature[0])
            motor_state.voltage = float(msg.motor_state[i].vol)
            motor_state.position = float(msg.motor_state[i].q)
            motor_state.velocity = float(msg.motor_state[i].dq)
            motor_list_msg.motor_list.append(motor_state)
            positions.append(motor_state.position)
        self.left_gripper_state_publisher.publish(motor_list_msg)

        if  self.send_commands:
            return
        self.get_logger().debug(f'Left hand positions: {positions}')

    def right_callback(self, msg: HandState_):
        motor_list_msg = MotorStateList()
        positions = []
        for i in range(len(msg.motor_state)):
            motor_state = MotorState()
            motor_state.name = f"right_motor_{i}"
            motor_state.temperature = float(msg.motor_state[i].temperature[0])
            motor_state.voltage = float(msg.motor_state[i].vol)
            motor_state.position = float(msg.motor_state[i].q)
            motor_state.velocity = float(msg.motor_state[i].dq)
            motor_list_msg.motor_list.append(motor_state)
            positions.append(motor_state.position)
        self.right_gripper_state_publisher.publish(motor_list_msg)

        if  self.send_commands:
            return
        self.get_logger().debug(f'Right hand positions: {positions}')

    def create_cmd(self, values):
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(self.total_motors):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q = values[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 1.5
            cmd.motor_cmd[i].kd = 0.2
        return cmd

    def publish_commands(self):
        if not self.send_commands:
            return
        if hasattr(self, "right_pub") and self.right_action is not None:
            self.right_pub.Write(self.create_cmd(self.right_target))
        if hasattr(self, "left_pub") and self.left_action is not None:
            self.left_pub.Write(self.create_cmd(self.left_target))

def main(args=None):
    rclpy.init(args=args)
    node = DX3Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
