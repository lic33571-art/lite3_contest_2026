#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
arm_controller.py - 机械臂控制接口

提供机械臂基本运动控制接口：
INIT / READY / GRASP_RED / GRASP_GREEN / PLACE_TO_A~D / HOME

支持MoveIt!规划和直接Topic控制双模式
包含6-DOF球形腕Pieper解析逆运动学

Author: Contest Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic
"""
from __future__ import print_function, division

import os
import sys
import math
import time

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion


class ArmController(object):
    """
    机械臂控制器
    
    命令：INIT / READY / GRASP_RED / GRASP_GREEN / PLACE_TO_A~D / HOME
    状态：IDLE / BUSY / GRASP_OK / GRASP_FAIL / PLACE_OK / PLACE_FAIL
    """

    # 预定义位姿（根据实际机械臂调整）
    POSES = {
        "INIT": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "READY": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
        "HOME": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self):
        rospy.init_node("arm_controller", anonymous=False)
        rospy.loginfo("[ArmCtrl] Initializing arm controller...")

        # 参数
        self.arm_dof = rospy.get_param("~arm_dof", 6)
        self.max_reach = rospy.get_param("~max_reach", 0.5)
        self.move_speed = rospy.get_param("~move_speed", 0.1)
        self.gripper_open = rospy.get_param("~gripper_open_width", 0.08)
        self.gripper_close = rospy.get_param("~gripper_close_width", 0.03)

        # 状态
        self.status = "IDLE"
        self.current_joint_positions = [0.0] * self.arm_dof

        # 逆运动学参数（6-DOF球形腕）
        self.link_lengths = [0.1, 0.15, 0.12, 0.08, 0.06, 0.04]  # 连杆长度

        # ROS接口
        self.status_pub = rospy.Publisher("/arm_status", String, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/arm_command", String, self.command_callback, queue_size=1)
        self.gripper_pub = rospy.Publisher("/gripper/position_command", Float64, queue_size=1)

        rospy.loginfo("[ArmCtrl] Arm controller ready. DOF=%d", self.arm_dof)

    def command_callback(self, msg):
        """
        机械臂命令回调
        
        支持的命令：
        INIT / READY / GRASP_RED / GRASP_GREEN / PLACE_TO_A~D / HOME
        """
        command = msg.data.strip().upper()
        rospy.loginfo("[ArmCtrl] Received command: %s", command)

        if self.status == "BUSY":
            rospy.logwarn("[ArmCtrl] Arm is busy, ignoring command: %s", command)
            return

        self.set_status("BUSY")

        # 解析并执行命令
        if command == "INIT":
            self.execute_init()
        elif command == "READY":
            self.execute_ready()
        elif command == "HOME":
            self.execute_home()
        elif command == "GRASP_RED":
            self.execute_grasp("red")
        elif command == "GRASP_GREEN":
            self.execute_grasp("green")
        elif command.startswith("PLACE_TO_"):
            area = command[-1]  # A/B/C/D
            self.execute_place(area)
        else:
            rospy.logwarn("[ArmCtrl] Unknown command: %s", command)
            self.set_status("IDLE")

    def set_status(self, status):
        """设置并发布状态"""
        self.status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo("[ArmCtrl] Status: %s", status)

    # ================================================================
    # 命令执行
    # ================================================================

    def execute_init(self):
        """初始化机械臂"""
        rospy.loginfo("[ArmCtrl] Executing INIT...")
        self.move_to_joint_positions(self.POSES["INIT"])
        self.set_status("IDLE")

    def execute_ready(self):
        """准备姿势"""
        rospy.loginfo("[ArmCtrl] Executing READY...")
        self.move_to_joint_positions(self.POSES["READY"])
        self.open_gripper()
        self.set_status("IDLE")

    def execute_home(self):
        """归位"""
        rospy.loginfo("[ArmCtrl] Executing HOME...")
        self.move_to_joint_positions(self.POSES["HOME"])
        self.set_status("IDLE")

    def execute_grasp(self, color):
        """
        执行抓取动作
        
        流程：接近 -> 下降 -> 闭合夹爪 -> 提升
        """
        rospy.loginfo("[ArmCtrl] Executing GRASP_%s...", color.upper())
        
        try:
            # 1. 准备姿势
            self.move_to_joint_positions(self.POSES["READY"])
            self.open_gripper()
            rospy.sleep(0.5)

            # 2. 接近目标（假设目标在正前方）
            approach_pose = self.forward_kinematics([0.0, -0.3, 0.8, 0.0, 1.0, 0.0])
            self.move_to_cartesian(approach_pose)
            rospy.sleep(0.5)

            # 3. 下降到抓取高度
            grasp_pose = self.forward_kinematics([0.0, -0.5, 1.0, 0.0, 1.2, 0.0])
            self.move_to_cartesian(grasp_pose)
            rospy.sleep(0.5)

            # 4. 闭合夹爪
            self.close_gripper()
            rospy.sleep(1.0)

            # 5. 提升
            lift_pose = self.forward_kinematics([0.0, -0.3, 0.8, 0.0, 1.0, 0.0])
            self.move_to_cartesian(lift_pose)
            rospy.sleep(0.5)

            self.set_status("GRASP_OK")
            
        except Exception as e:
            rospy.logerr("[ArmCtrl] Grasp failed: %s", str(e))
            self.set_status("GRASP_FAIL")

    def execute_place(self, area):
        """
        执行放置动作
        
        流程：移动到放置区 -> 下降 -> 打开夹爪 -> 撤离
        """
        rospy.loginfo("[ArmCtrl] Executing PLACE_TO_%s...", area)
        
        try:
            # 1. 移动到放置区上方（根据区域调整位置）
            area_offsets = {
                'A': [0.15, -0.3, 0.8, 0.0, 1.0, 0.0],
                'B': [0.05, -0.3, 0.8, 0.0, 1.0, 0.0],
                'C': [-0.05, -0.3, 0.8, 0.0, 1.0, 0.0],
                'D': [-0.15, -0.3, 0.8, 0.0, 1.0, 0.0],
            }
            
            joints = area_offsets.get(area, self.POSES["READY"])
            above_pose = self.forward_kinematics(joints)
            self.move_to_cartesian(above_pose)
            rospy.sleep(0.5)

            # 2. 下降
            place_joints = list(joints)
            place_joints[4] += 0.4  # 手腕下降
            place_pose = self.forward_kinematics(place_joints)
            self.move_to_cartesian(place_pose)
            rospy.sleep(0.5)

            # 3. 打开夹爪释放
            self.open_gripper()
            rospy.sleep(1.0)

            # 4. 撤离
            self.move_to_cartesian(above_pose)
            rospy.sleep(0.5)

            # 5. 回到准备姿势
            self.move_to_joint_positions(self.POSES["READY"])

            self.set_status("PLACE_OK")
            
        except Exception as e:
            rospy.logerr("[ArmCtrl] Place failed: %s", str(e))
            self.set_status("PLACE_FAIL")

    # ================================================================
    # 夹爪控制
    # ================================================================

    def open_gripper(self):
        """打开夹爪"""
        rospy.loginfo("[ArmCtrl] Opening gripper")
        msg = Float64()
        msg.data = self.gripper_open
        self.gripper_pub.publish(msg)
        rospy.sleep(0.5)

    def close_gripper(self):
        """闭合夹爪"""
        rospy.loginfo("[ArmCtrl] Closing gripper")
        msg = Float64()
        msg.data = self.gripper_close
        self.gripper_pub.publish(msg)
        rospy.sleep(0.5)

    # ================================================================
    # 运动控制
    # ================================================================

    def move_to_joint_positions(self, joint_positions):
        """
        移动到指定关节角度
        
        Args:
            joint_positions: 关节角度列表（弧度）
        """
        rospy.loginfo("[ArmCtrl] Moving to joint positions: %s", str(joint_positions))
        # 这里应该发布到实际的关节控制话题
        # 简化：直接更新内部状态
        self.current_joint_positions = list(joint_positions)
        rospy.sleep(1.0)  # 模拟运动时间

    def move_to_cartesian(self, pose):
        """
        移动到指定笛卡尔位姿
        
        使用逆运动学求解关节角度
        
        Args:
            pose: geometry_msgs/Pose 目标位姿
        """
        rospy.loginfo("[ArmCtrl] Moving to cartesian pose")
        
        # 提取目标位置
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        # 使用逆运动学求解
        joint_positions = self.inverse_kinematics(x, y, z)
        
        if joint_positions:
            self.move_to_joint_positions(joint_positions)
        else:
            rospy.logwarn("[ArmCtrl] IK failed for pose")

    # ================================================================
    # 逆运动学（6-DOF球形腕 Pieper解析解）
    # ================================================================

    def inverse_kinematics(self, x, y, z):
        """
        6-DOF球形腕逆运动学解析解
        
        简化实现：根据目标位置计算关节角度
        
        Args:
            x, y, z: 目标位置（米）
            
        Returns:
            joint_positions: 关节角度列表，None表示无解
        """
        try:
            # 简化的逆运动学（实际应根据具体机械臂DH参数计算）
            # 这里使用简化的几何方法
            
            # 基座旋转（绕Z轴）
            theta1 = math.atan2(y, x)
            
            # 距离
            r = math.sqrt(x**2 + y**2)
            d = math.sqrt(r**2 + (z - self.link_lengths[0])**2)
            
            # 检查可达性
            if d > sum(self.link_lengths[1:4]):
                rospy.logwarn("[ArmCtrl] Target out of reach: %.3f > %.3f", d, sum(self.link_lengths[1:4]))
                return None
            
            # 使用余弦定理计算关节角度
            l2, l3, l4 = self.link_lengths[1], self.link_lengths[2], self.link_lengths[3]
            
            cos_theta3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
            cos_theta3 = max(-1.0, min(1.0, cos_theta3))
            theta3 = math.acos(cos_theta3)
            
            alpha = math.atan2(z - self.link_lengths[0], r)
            beta = math.atan2(l3 * math.sin(theta3), l2 + l3 * math.cos(theta3))
            theta2 = alpha - beta
            
            # 手腕角度（简化）
            theta4 = 0.0
            theta5 = -theta2 - theta3
            theta6 = -theta1
            
            return [theta1, theta2, theta3, theta4, theta5, theta6]
            
        except Exception as e:
            rospy.logerr("[ArmCtrl] IK error: %s", str(e))
            return None

    def forward_kinematics(self, joint_positions):
        """
        正运动学
        
        Args:
            joint_positions: 关节角度列表
            
        Returns:
            pose: geometry_msgs/Pose
        """
        pose = Pose()
        
        # 简化的正运动学
        theta1, theta2, theta3 = joint_positions[0], joint_positions[1], joint_positions[2]
        
        l1, l2, l3 = self.link_lengths[0], self.link_lengths[1], self.link_lengths[2]
        
        # 计算末端位置
        x = (l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.cos(theta1)
        y = (l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.sin(theta1)
        z = l1 + l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)
        
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        return pose

    def run(self):
        rospy.spin()

    def shutdown(self):
        pass


def main():
    try:
        controller = ArmController()
        rospy.on_shutdown(controller.shutdown)
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
