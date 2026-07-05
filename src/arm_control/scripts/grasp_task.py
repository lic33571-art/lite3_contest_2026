#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
grasp_task.py - 抓取+放置任务逻辑

基于颜色的物体检测（红色/绿色）
深度相机辅助定位3D抓取位姿
抓取-移动-放置完整pipeline

Author: Contest Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic
"""
from __future__ import print_function, division

import os
import sys
import math

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from cv_bridge import CvBridge


class GraspTask(object):
    """
    抓取任务执行器
    
    功能：
    1. 基于颜色的长条检测（红色/绿色HSV）
    2. 深度相机3D定位
    3. 抓取+放置任务编排
    """

    def __init__(self):
        rospy.init_node("grasp_task", anonymous=False)
        rospy.loginfo("[GraspTask] Initializing grasp task node...")

        # 参数
        self.grasp_height = rospy.get_param("~grasp_height", 0.55)
        self.grasp_approach = rospy.get_param("~grasp_approach_distance", 0.1)
        self.move_speed = rospy.get_param("~move_speed", 0.1)

        # HSV颜色阈值
        self.red_lower1 = tuple(rospy.get_param("~red_hsv_lower", [0, 100, 100]))
        self.red_upper1 = tuple(rospy.get_param("~red_hsv_upper", [10, 255, 255]))
        self.red_lower2 = tuple(rospy.get_param("~red_hsv_lower2", [160, 100, 100]))
        self.red_upper2 = tuple(rospy.get_param("~red_hsv_upper2", [180, 255, 255]))
        self.green_lower = tuple(rospy.get_param("~green_hsv_lower", [35, 50, 50]))
        self.green_upper = tuple(rospy.get_param("~green_hsv_upper", [85, 255, 255]))

        # 状态
        self.current_task = None
        self.arm_status = "IDLE"

        # ROS接口
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1)
        
        self.target_pub = rospy.Publisher("/grasp_target_pose", PoseStamped, queue_size=1)
        self.arm_cmd_pub = rospy.Publisher("/arm_command", String, queue_size=1)
        self.status_pub = rospy.Publisher("/grasp_task/status", String, queue_size=1)

        self.current_image = None
        self.current_depth = None

        rospy.loginfo("[GraspTask] Grasp task node ready.")

    def image_callback(self, msg):
        """彩色图像回调"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("[GraspTask] Image error: %s", str(e))

    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            rospy.logwarn("[GraspTask] Depth error: %s", str(e))

    def detect_object(self, color="red"):
        """
        基于颜色检测长条物体
        
        Args:
            color: "red" 或 "green"
            
        Returns:
            (cx, cy, depth): 物体中心像素坐标和深度，None表示未检测到
        """
        if self.current_image is None:
            return None

        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

        if color == "red":
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None

        # 找最大的轮廓
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:  # 最小面积过滤
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 获取深度
        depth = None
        if self.current_depth is not None:
            h, w = self.current_depth.shape
            if 0 <= cy < h and 0 <= cx < w:
                depth = self.current_depth[cy, cx]
                if math.isnan(depth) or depth <= 0:
                    depth = None

        return (cx, cy, depth)

    def pixel_to_3d(self, px, py, depth):
        """
        像素坐标+深度转3D坐标
        
        简化模型：假设针孔相机模型
        
        Args:
            px, py: 像素坐标
            depth: 深度值（米）
            
        Returns:
            (x, y, z): 相机坐标系下的3D坐标
        """
        # RealSense D435i 内参（近似值）
        fx, fy = 605.0, 605.0  # 焦距
        cx, cy = 320.0, 240.0  # 光心

        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth

        return (x, y, z)

    def execute_grasp_and_place(self, color, target_area):
        """
        执行一次完整的抓取+放置
        
        Args:
            color: "red" 或 "green"
            target_area: "A"/"B"/"C"/"D"
            
        Returns:
            bool: 是否成功
        """
        rospy.loginfo("[GraspTask] Executing grasp(%s) -> place(%s)", color, target_area)

        # 步骤1: 检测物体
        obj = self.detect_object(color)
        if obj is None:
            rospy.logwarn("[GraspTask] Object not found: %s", color)
            return False

        px, py, depth = obj
        rospy.loginfo("[GraspTask] Object at (%d, %d), depth=%s", px, py, str(depth))

        # 步骤2: 计算3D位置
        if depth is not None:
            x, y, z = self.pixel_to_3d(px, py, depth)
            rospy.loginfo("[GraspTask] 3D position: (%.3f, %.3f, %.3f)", x, y, z)

        # 步骤3: 发送抓取命令
        cmd = "GRASP_" + color.upper()
        self.arm_cmd_pub.publish(String(data=cmd))
        rospy.sleep(15.0)  # 等待抓取完成

        # 步骤4: 发送放置命令
        cmd = "PLACE_TO_" + target_area
        self.arm_cmd_pub.publish(String(data=cmd))
        rospy.sleep(15.0)  # 等待放置完成

        rospy.loginfo("[GraspTask] Grasp and place cycle complete.")
        return True

    def run(self):
        rospy.spin()

    def shutdown(self):
        pass


def main():
    try:
        task = GraspTask()
        rospy.on_shutdown(task.shutdown)
        task.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
