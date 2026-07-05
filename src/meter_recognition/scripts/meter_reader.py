#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
meter_reader.py - 仪表盘指针识别模块

功能：
1. 霍夫圆检测定位仪表盘
2. Canny边缘 + 霍夫直线检测识别指针
3. 颜色掩膜区分黄/绿/红三个扇区
4. 根据指针角度判断状态：偏低(黄)/正常(绿)/偏高(红)

Author: Robot Vision Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic, OpenCV 4.x
"""
from __future__ import print_function, division

import os
import sys
import math
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class MeterReader(object):
    """
    仪表盘指针识别类
    
    识别流程：
    1. 图像预处理（灰度化、高斯模糊）
    2. 霍夫圆检测定位仪表盘
    3. Canny边缘检测 + 霍夫直线检测指针
    4. 计算指针角度
    5. 颜色扇区判断状态
    """

    def __init__(self):
        rospy.init_node("meter_reader", anonymous=False)
        rospy.loginfo("[MeterReader] Initializing meter reader node...")

        # ============================================================
        # 参数加载
        # ============================================================
        self.meter_min_radius = rospy.get_param("~meter_min_radius", 30)
        self.meter_max_radius = rospy.get_param("~meter_max_radius", 150)
        self.pointer_length_ratio = rospy.get_param("~pointer_length_ratio", 0.7)
        
        # 颜色分区角度（相对于圆心水平向右为0度，顺时针为正）
        self.yellow_sector = (30, 90)    # 偏低 - 黄色区域
        self.green_sector = (90, 150)    # 正常 - 绿色区域
        self.red_sector = (150, 210)     # 偏高 - 红色区域
        
        rospy.loginfo("[MeterReader] Parameters:")
        rospy.loginfo("  meter_min_radius: %d", self.meter_min_radius)
        rospy.loginfo("  meter_max_radius: %d", self.meter_max_radius)
        rospy.loginfo("  pointer_length_ratio: %.2f", self.pointer_length_ratio)

        # ============================================================
        # ROS接口
        # ============================================================
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )
        self.debug_pub = rospy.Publisher("/meter_recognition/debug", Image, queue_size=1)

        rospy.loginfo("[MeterReader] Node ready.")

    def image_callback(self, msg):
        """相机图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn("[MeterReader] CvBridge error: %s", str(e))
            return

        # 执行识别
        status, confidence, debug_image = self.read_meter(cv_image)
        
        if status:
            rospy.loginfo("[MeterReader] Result: %s (confidence: %.2f)", status, confidence)

    def read_meter(self, image):
        """
        读取仪表盘状态
        
        Args:
            image: BGR格式图像
            
        Returns:
            status: 状态字符串（"正常"/"偏低"/"偏高"/None）
            confidence: 置信度0.0~1.0
            debug_image: 调试图像
        """
        # 步骤1: 检测仪表盘圆形区域
        circles = self.detect_circles(image)
        if not circles:
            return None, 0.0, image

        best_result = None
        best_confidence = 0.0
        best_debug = image.copy()

        for circle in circles:
            cx, cy, radius = circle
            
            # 步骤2: 提取仪表盘ROI
            meter_roi = self.extract_meter_roi(image, cx, cy, radius)
            
            # 步骤3: 检测指针
            pointer_angle = self.detect_pointer(meter_roi, radius)
            if pointer_angle is None:
                continue
            
            # 步骤4: 判断颜色扇区
            status, confidence = self.angle_to_status(pointer_angle)
            
            if confidence > best_confidence:
                best_confidence = confidence
                best_result = status
                best_debug = self.draw_debug(image, cx, cy, radius, pointer_angle, status)

        return best_result, best_confidence, best_debug

    def detect_circles(self, image):
        """
        霍夫圆检测
        
        Args:
            image: BGR图像
            
        Returns:
            circles: 圆列表，每个元素为 (cx, cy, radius)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 霍夫圆检测
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=100,
            param2=30,
            minRadius=self.meter_min_radius,
            maxRadius=self.meter_max_radius
        )
        
        if circles is None:
            return []
        
        result = []
        for c in circles[0, :]:
            cx, cy, r = int(c[0]), int(c[1]), int(c[2])
            result.append((cx, cy, r))
        
        return result

    def extract_meter_roi(self, image, cx, cy, radius):
        """
        提取仪表盘ROI
        
        Args:
            image: 原图像
            cx, cy, radius: 圆心和半径
            
        Returns:
            roi: 仪表盘区域图像
            mask: 圆形掩膜
        """
        h, w = image.shape[:2]
        
        # 边界检查
        x1 = max(0, cx - radius)
        y1 = max(0, cy - radius)
        x2 = min(w, cx + radius)
        y2 = min(h, cy + radius)
        
        roi = image[y1:y2, x1:x2].copy()
        
        # 创建圆形掩膜
        mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        local_cx = radius
        local_cy = radius
        cv2.circle(mask, (local_cx, local_cy), radius - 5, 255, -1)
        
        # 应用掩膜
        roi_masked = cv2.bitwise_and(roi, roi, mask=mask)
        
        return roi_masked

    def detect_pointer(self, meter_roi, radius):
        """
        检测指针角度
        
        方法：Canny边缘 + 霍夫直线检测，筛选过圆心的直线
        
        Args:
            meter_roi: 仪表盘ROI
            radius: 仪表盘半径
            
        Returns:
            angle: 指针角度（度），None表示未检测到
        """
        gray = cv2.cvtColor(meter_roi, cv2.COLOR_BGR2GRAY)
        
        # Canny边缘检测
        edges = cv2.Canny(gray, 50, 150)
        
        # 霍夫直线检测
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 
                               threshold=30, 
                               minLineLength=int(radius * 0.5),
                               maxLineGap=10)
        
        if lines is None:
            return None
        
        # 圆心（ROI中心）
        cx = meter_roi.shape[1] // 2
        cy = meter_roi.shape[0] // 2
        
        best_line = None
        best_score = 0
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 计算直线到圆心的距离
            line_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if line_len < radius * 0.3:
                continue
            
            # 计算直线中点到圆心的距离
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            dist_to_center = math.sqrt((mid_x - cx) ** 2 + (mid_y - cy) ** 2)
            
            # 过圆心的直线得分更高
            score = line_len - dist_to_center * 2
            
            if score > best_score:
                best_score = score
                best_line = (x1, y1, x2, y2)
        
        if best_line is None:
            return None
        
        # 计算指针角度（从圆心到线段端点的方向）
        x1, y1, x2, y2 = best_line
        
        # 选择与圆心较近的端点作为指针根部
        d1 = (x1 - cx) ** 2 + (y1 - cy) ** 2
        d2 = (x2 - cx) ** 2 + (y2 - cy) ** 2
        
        if d1 < d2:
            # (x1, y1) 是根部，(x2, y2) 是指针尖
            dx = x2 - cx
            dy = y2 - cy
        else:
            dx = x1 - cx
            dy = y1 - cy
        
        # 计算角度（atan2返回弧度，转换为度）
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        
        # 归一化到0-360
        if angle_deg < 0:
            angle_deg += 360
        
        return angle_deg

    def angle_to_status(self, angle):
        """
        将指针角度转换为状态
        
        分区定义：
        - 黄色区域(30-90度): 偏低
        - 绿色区域(90-150度): 正常
        - 红色区域(150-210度): 偏高
        
        Args:
            angle: 指针角度（度）
            
        Returns:
            status: 状态字符串
            confidence: 置信度
        """
        # 黄色区域 - 偏低
        if self.yellow_sector[0] <= angle < self.yellow_sector[1]:
            # 计算置信度（越靠近中心越高）
            center = (self.yellow_sector[0] + self.yellow_sector[1]) / 2
            confidence = 1.0 - abs(angle - center) / (self.yellow_sector[1] - self.yellow_sector[0])
            return "偏低", max(0.5, confidence)
        
        # 绿色区域 - 正常
        elif self.green_sector[0] <= angle < self.green_sector[1]:
            center = (self.green_sector[0] + self.green_sector[1]) / 2
            confidence = 1.0 - abs(angle - center) / (self.green_sector[1] - self.green_sector[0])
            return "正常", max(0.7, confidence)
        
        # 红色区域 - 偏高
        elif self.red_sector[0] <= angle < self.red_sector[1]:
            center = (self.red_sector[0] + self.red_sector[1]) / 2
            confidence = 1.0 - abs(angle - center) / (self.red_sector[1] - self.red_sector[0])
            return "偏高", max(0.5, confidence)
        
        else:
            # 未知区域
            return None, 0.0

    def draw_debug(self, image, cx, cy, radius, angle, status):
        """
        绘制调试图像
        
        Args:
            image: 原图像
            cx, cy, radius: 仪表盘圆心和半径
            angle: 指针角度
            status: 识别状态
            
        Returns:
            debug_image: 调试图像
        """
        debug = image.copy()
        
        # 画圆
        cv2.circle(debug, (cx, cy), radius, (0, 255, 0), 2)
        cv2.circle(debug, (cx, cy), 3, (0, 0, 255), -1)
        
        # 画指针方向
        angle_rad = math.radians(angle)
        ptr_x = int(cx + radius * 0.8 * math.cos(angle_rad))
        ptr_y = int(cy + radius * 0.8 * math.sin(angle_rad))
        cv2.line(debug, (cx, cy), (ptr_x, ptr_y), (0, 0, 255), 3)
        
        # 画扇区参考线
        for sector_name, sector_range, color in [
            ("LOW", self.yellow_sector, (0, 255, 255)),
            ("OK", self.green_sector, (0, 255, 0)),
            ("HIGH", self.red_sector, (0, 0, 255))
        ]:
            start_angle = sector_range[0]
            end_angle = sector_range[1]
            # 绘制扇区弧线
            cv2.ellipse(debug, (cx, cy), (radius + 10, radius + 10),
                       0, start_angle, end_angle, color, 2)
        
        # 状态文字
        color_map = {
            "正常": (0, 255, 0),
            "偏低": (0, 255, 255),
            "偏高": (0, 0, 255)
        }
        color = color_map.get(status, (128, 128, 128))
        cv2.putText(debug, "Status: %s" % status, (cx - 50, cy - radius - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(debug, "Angle: %.1f deg" % angle, (cx - 50, cy - radius - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return debug

    def run(self):
        """主循环"""
        rospy.spin()

    def shutdown(self):
        """关闭"""
        pass


def main():
    try:
        reader = MeterReader()
        rospy.on_shutdown(reader.shutdown)
        reader.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
