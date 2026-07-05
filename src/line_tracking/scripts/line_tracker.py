#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
line_tracker.py - Optimized Line Tracking Module (V31+)

基于V31优化的国赛巡线代码，适配六条均匀白线段地图。
功能：白线段检测 + 黑色边界线避障 + 锥形桶避障 + PD巡线控制

Author: Robot Vision Team
Version: 3.1+
Compatible: Python 2/3, ROS Noetic, OpenCV 4.x
"""
from __future__ import print_function, division

import os
import sys
import math
import collections
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge, CvBridgeError

# Python 2/3 compatibility
try:
    import queue
except ImportError:
    import Queue as queue


class LineTracker(object):
    """
    巡线导航核心类 - V31+优化版
    
    算法流程：
    1. ROI截取：画面下方50%~95%区域
    2. 白线段检测：HSV高亮度+低饱和度阈值，形态学操作
    3. 聚类过滤：去除间距>100px的孤立线段
    4. 中线计算：奇数条取中间，偶数条取中间两条中点
    5. 黑色边界线检测：V<mean_v-25，竖向形态学操作
    6. 避障力计算：分层避障（安全区/危险区/预警区）
    7. 巡线控制：PD控制器，死区处理
    8. LOST状态：左右扫描找线
    9. 冲出保护：从"有黑线"变"无黑线"时回退
    10. 锥形桶避障：HSV检测红色锥形桶
    """

    def __init__(self):
        rospy.init_node("line_tracker", anonymous=False)
        rospy.loginfo("[LineTracker] V31+ optimized line tracking node initializing...")

        # ============================================================
        # 参数加载（全部从rospy.get_param()读取，带默认值）
        # ============================================================
        # ROI参数
        self.roi_top = rospy.get_param("~roi_top", 0.50)
        self.roi_bottom = rospy.get_param("~roi_bottom", 0.95)

        # 速度参数
        self.linear_x = rospy.get_param("~linear_x", 0.10)
        self.linear_x_min = rospy.get_param("~linear_x_min", 0.06)

        # PD控制器参数
        self.kp_track = rospy.get_param("~kp_track", 0.004)
        self.kd_track = rospy.get_param("~kd_track", 0.003)
        self.dead_zone = rospy.get_param("~dead_zone", 20)
        self.history_size = rospy.get_param("~history_size", 5)

        # 聚类过滤参数
        self.cluster_max_gap = rospy.get_param("~cluster_max_gap", 100)

        # 黑色边界线避障参数
        self.black_safe_distance = rospy.get_param("~black_safe_distance", 120)
        self.black_danger_distance = rospy.get_param("~black_danger_distance", 80)
        self.black_warn_distance = rospy.get_param("~black_warn_distance", 45)
        self.avoid_kp = rospy.get_param("~avoid_kp", 0.008)
        self.avoid_weight = rospy.get_param("~avoid_weight", 2.5)
        self.avoid_max = rospy.get_param("~avoid_max", 0.40)

        # LOST扫描参数
        self.scan_angular = rospy.get_param("~scan_angular", 0.20)
        self.scan_period = rospy.get_param("~scan_period", 40)
        self.scan_speed = rospy.get_param("~scan_speed", 0.04)

        # 冲出保护参数
        self.outbound_recover_angular = rospy.get_param("~outbound_recover_angular", 0.25)

        # 白线段检测参数（国赛特色）
        self.white_line_min_area = rospy.get_param("~white_line_min_area", 40)
        self.white_line_aspect_min = rospy.get_param("~white_line_aspect_min", 1.0)
        self.white_line_aspect_max = rospy.get_param("~white_line_aspect_max", 50.0)

        # 锥形桶避障参数
        self.cone_detect_enabled = rospy.get_param("~cone_detect_enabled", True)
        self.cone_red_lower = tuple(rospy.get_param("~cone_red_lower", [0, 100, 100]))
        self.cone_red_upper = tuple(rospy.get_param("~cone_red_upper", [10, 255, 255]))
        self.cone_red_lower2 = tuple(rospy.get_param("~cone_red_lower2", [160, 100, 100]))
        self.cone_red_upper2 = tuple(rospy.get_param("~cone_red_upper2", [180, 255, 255]))
        self.cone_min_area = rospy.get_param("~cone_min_area", 500)
        self.cone_avoid_distance = rospy.get_param("~cone_avoid_distance", 150)
        self.cone_avoid_angular = rospy.get_param("~cone_avoid_angular", 0.3)

        # 形态学核（预创建，避免每帧重复分配）
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 5))
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.kernel_black = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 15))

        # ============================================================
        # 状态变量
        # ============================================================
        self.bridge = CvBridge()
        self.current_state = "INIT"           # INIT / TRACKING / LOST / AVOIDING / OUTBOUND
        self.prev_error = 0.0
        self.guidance_x_history = collections.deque(maxlen=self.history_size)
        self.last_guidance_x = None

        # 黑色边界线状态（用于冲出保护）
        self.black_detected_prev = False      # 上一帧是否检测到黑线
        self.outbound_recover_frames = 0      # 冲出保护回退帧计数
        self.outbound_recover_duration = 15   # 回退持续帧数

        # LOST扫描状态
        self.scan_direction = 1               # 1=右扫, -1=左扫
        self.scan_counter = 0                 # 扫描帧计数器

        # 帧计数
        self.frame_count = 0

        # 锥形桶状态
        self.cone_detected = False
        self.cone_cx = 0
        self.cone_cy = 0

        # 调试可视化开关
        self.debug_viz = rospy.get_param("~debug_viz", True)

        rospy.loginfo("[LineTracker] Parameters loaded successfully.")
        rospy.loginfo("[LineTracker]  ROI: [%.2f, %.2f], Speed: %.3f/%.3f, KP: %.4f, KD: %.4f",
                      self.roi_top, self.roi_bottom, self.linear_x, self.linear_x_min,
                      self.kp_track, self.kd_track)

        # ============================================================
        # ROS通信接口
        # ============================================================
        # 订阅相机彩色图像
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        # 发布速度指令（由main_controller在LINE_TRACKING状态下转发）
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 发布状态（供main_controller查询）
        self.status_pub = rospy.Publisher("/line_tracking/status", String, queue_size=1)

        # 发布调试图像（可选）
        self.debug_img_pub = rospy.Publisher("/line_tracking/debug_image", Image, queue_size=1)

        # 状态查询服务（供main_controller查询当前状态）
        self.status_srv = rospy.Service(
            "/line_tracking/get_status",
            Trigger,
            self.handle_get_status
        )

        rospy.loginfo("[LineTracker] ROS interfaces initialized.")
        rospy.loginfo("[LineTracker] Node ready. Waiting for camera images...")

    def handle_get_status(self, req):
        """
        状态查询服务回调 - /line_tracking/get_status
        
        返回当前巡线状态，供main_controller查询使用。
        
        Returns:
            TriggerResponse: success=True, message=当前状态字符串
        """
        resp = TriggerResponse()
        resp.success = True
        resp.message = self.current_state
        return resp

    # ================================================================
    # ROS回调函数
    # ================================================================

    def image_callback(self, msg):
        """
        相机图像回调函数
        
        每帧执行完整巡线pipeline：
        1. 图像转换
        2. 白线段检测
        3. 黑色边界线避障
        4. 锥形桶避障
        5. PD巡线控制
        6. 状态机管理
        7. 速度指令发布
        8. 调试图像发布
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn("[LineTracker] CvBridge error: %s", str(e))
            return

        self.frame_count += 1

        # 执行巡线pipeline
        twist, debug_image = self.process_frame(cv_image)

        # 发布速度指令
        self.cmd_vel_pub.publish(twist)

        # 发布状态
        status_msg = String()
        status_msg.data = self.current_state
        self.status_pub.publish(status_msg)

        # 发布调试图像
        if self.debug_viz and debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_img_pub.publish(debug_msg)
            except CvBridgeError:
                pass

    # ================================================================
    # 核心处理pipeline
    # ================================================================

    def process_frame(self, image):
        """
        单帧处理pipeline - 巡线核心
        
        Args:
            image: BGR格式的OpenCV图像 (H x W x 3)
            
        Returns:
            twist: geometry_msgs/Twist 速度指令
            debug_image: BGR调试图像（用于可视化）
        """
        h, w = image.shape[:2]
        roi_h_start = int(h * self.roi_top)
        roi_h_end = int(h * self.roi_bottom)
        roi = image[roi_h_start:roi_h_end, :]
        roi_h, roi_w = roi.shape[:2]
        center_x = roi_w // 2

        # ============================================================
        # 步骤1: 白线段检测（国赛地图特色 - 六条均匀白线段）
        # ============================================================
        white_segments = self.detect_white_segments(roi, roi_w, roi_h)

        # 步骤2: 聚类过滤（去除孤立线段）
        filtered_segments = self.filter_isolated_segments(white_segments, roi_w)

        # 步骤3: 中线计算
        guidance_x, num_lines = self.compute_guidance_point(filtered_segments)

        # ============================================================
        # 步骤4: 黑色边界线检测（禁区避障）
        # ============================================================
        black_lines = self.detect_black_border(roi, roi_w, roi_h)
        black_detected = len(black_lines) > 0

        # 计算避障力
        avoid_angular, nearest_black_dist, nearest_black_side = self.compute_avoidance(
            black_lines, roi_w
        )

        # ============================================================
        # 步骤5: 锥形桶检测（红色锥形桶避障）
        # ============================================================
        cone_angular = 0.0
        if self.cone_detect_enabled:
            cone_angular = self.detect_and_avoid_cones(roi, roi_w, roi_h)

        # ============================================================
        # 步骤6: 状态机 + 巡线控制
        # ============================================================
        twist = Twist()

        # 冲出保护检测（从"有黑线"变"无黑线"且距离近）
        outbound_triggered = False
        if self.black_detected_prev and not black_detected:
            if nearest_black_dist < 100:  # 之前很近突然消失 = 可能冲出
                outbound_triggered = True
                self.outbound_recover_frames = self.outbound_recover_duration
                rospy.logwarn("[LineTracker] OUTBOUND detected! Recovering...")

        self.black_detected_prev = black_detected

        # 如果正在冲出恢复
        if self.outbound_recover_frames > 0:
            self.current_state = "OUTBOUND"
            self.outbound_recover_frames -= 1
            # 向反方向转向回退
            recover_direction = -nearest_black_side if nearest_black_side != 0 else 1
            twist.angular.z = recover_direction * self.outbound_recover_angular
            twist.linear.x = 0.02  # 极低速度前进
            
        # 如果检测到锥形桶且距离近
        elif abs(cone_angular) > 0.01:
            self.current_state = "CONE_AVOID"
            twist.angular.z = cone_angular
            twist.linear.x = 0.04  # 减速

        # 正常巡线/LOST状态
        else:
            if guidance_x is not None and num_lines >= 1:
                # ===== TRACKING状态 =====
                self.current_state = "TRACKING"
                
                # PD巡线控制
                error = center_x - guidance_x  # 误差：正=目标在右，需左转
                
                if abs(error) < self.dead_zone:
                    # 死区内：直行
                    angular_cmd = 0.0
                else:
                    # PD控制
                    self.guidance_x_history.append(error)
                    smoothed_error = sum(self.guidance_x_history) / len(self.guidance_x_history)
                    angular_cmd = (self.kp_track * smoothed_error + 
                                  self.kd_track * (smoothed_error - self.prev_error))
                    angular_cmd = max(-0.25, min(0.25, angular_cmd))
                
                self.prev_error = error if abs(error) >= self.dead_zone else 0
                
                # 速度根据线段数量自适应
                speed = self.linear_x_min + (self.linear_x - self.linear_x_min) * min(num_lines / 6.0, 1.0)
                speed = max(self.linear_x_min, speed)
                
                # 叠加避障力
                final_angular = angular_cmd + avoid_angular * self.avoid_weight
                
                # 预警区：直接停 + 猛转
                if black_detected and nearest_black_dist < self.black_warn_distance:
                    speed = 0.02
                    final_angular = self.avoid_max * nearest_black_side
                    rospy.logwarn_throttle(1.0, "[LineTracker] WARN STOP! dist=%d", nearest_black_dist)
                # 危险区：猛刹车
                elif black_detected and nearest_black_dist < self.black_danger_distance:
                    speed = 0.03
                    rospy.logwarn_throttle(1.0, "[LineTracker] DANGER BRAKE! dist=%d", nearest_black_dist)
                # 单边黑线预警
                elif black_detected and len(black_lines) == 1:
                    speed = min(speed, 0.04)
                
                # 角速度限幅
                final_angular = max(-0.50, min(0.50, final_angular))
                
                twist.linear.x = speed
                twist.angular.z = final_angular
                
            else:
                # ===== LOST状态 =====
                self.current_state = "LOST"
                self.scan_counter += 1
                
                # 切换扫描方向
                if self.scan_counter >= self.scan_period:
                    self.scan_counter = 0
                    self.scan_direction *= -1
                
                # LOST时叠加避障力
                lost_angular = self.scan_direction * self.scan_angular
                if black_detected and avoid_angular != 0:
                    # 避障优先
                    if (avoid_angular > 0 and lost_angular > 0) or (avoid_angular < 0 and lost_angular < 0):
                        lost_angular += avoid_angular * self.avoid_weight
                    else:
                        lost_angular = avoid_angular * self.avoid_weight
                    twist.linear.x = 0.03
                else:
                    twist.linear.x = self.scan_speed
                
                twist.angular.z = lost_angular
                
                # 重置PD历史
                self.guidance_x_history.clear()
                self.prev_error = 0

        # ============================================================
        # 构建调试图像
        # ============================================================
        debug_image = self.build_debug_image(
            roi, roi_w, roi_h, center_x,
            filtered_segments, guidance_x, num_lines,
            black_lines, nearest_black_dist, nearest_black_side,
            twist
        )

        return twist, debug_image

    # ================================================================
    # 白线段检测
    # ================================================================

    def detect_white_segments(self, roi, roi_w, roi_h):
        """
        检测白线段 - 国赛地图特色（六条均匀白线段）
        
        算法：HSV色彩空间，高亮度区域+低饱和度，形态学操作
        
        Args:
            roi: ROI区域图像 (BGR)
            roi_w, roi_h: ROI宽高
            
        Returns:
            segments: 检测到的线段列表，每个元素为 {'center': (cx, cy), 'contour': cnt}
        """
        # 转换为HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        
        # 自适应亮度阈值：mean_v + 15
        mean_v = np.mean(v_ch)
        adaptive_threshold = int(mean_v + 15)
        adaptive_threshold = max(75, min(180, adaptive_threshold))
        
        # 高亮度区域（白线段很亮）
        _, bright_mask = cv2.threshold(v_ch, adaptive_threshold, 255, cv2.THRESH_BINARY)
        
        # 低饱和度（白色/银色饱和度低）
        gray_mask = cv2.inRange(s_ch, 0, 80)
        
        # 交集：亮且低饱和 = 白线
        white_mask = cv2.bitwise_and(bright_mask, gray_mask)
        
        # 形态学操作：闭运算连接断裂线段 -> 开运算去噪 -> 膨胀增强
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel_close)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel_open)
        white_mask = cv2.dilate(white_mask, self.kernel_dilate, iterations=1)
        
        # 查找轮廓
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        segments = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.white_line_min_area:
                continue
            
            # 长宽比检查（白线段是细长条状）
            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = float(max(bw, bh)) / max(min(bw, bh), 1)
            if aspect_ratio < self.white_line_aspect_min or aspect_ratio > self.white_line_aspect_max:
                continue
            
            # 计算中心点
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                segments.append({
                    'center': (cx, cy),
                    'contour': cnt,
                    'area': area,
                    'bbox': (x, y, bw, bh)
                })
        
        return segments

    # ================================================================
    # 聚类过滤
    # ================================================================

    def filter_isolated_segments(self, segments, roi_w):
        """
        聚类过滤 - 去除间距>cluster_max_gap的孤立线段
        
        国赛地图的六条白线段均匀分布，孤立的噪声线段间距大，可被过滤。
        
        Args:
            segments: 候选线段列表
            roi_w: ROI宽度
            
        Returns:
            filtered: 过滤后的线段列表
        """
        if len(segments) <= 2:
            return segments
        
        # 按x坐标排序
        sorted_segs = sorted(segments, key=lambda s: s['center'][0])
        n = len(sorted_segs)
        
        keep = []
        for i in range(n):
            cx = sorted_segs[i]['center'][0]
            min_gap = roi_w  # 初始化为最大值
            
            # 计算与左邻居的距离
            if i > 0:
                left_cx = sorted_segs[i - 1]['center'][0]
                min_gap = min(min_gap, abs(cx - left_cx))
            
            # 计算与右邻居的距离
            if i < n - 1:
                right_cx = sorted_segs[i + 1]['center'][0]
                min_gap = min(min_gap, abs(cx - right_cx))
            
            # 如果最近邻居距离在阈值内，保留
            if min_gap <= self.cluster_max_gap:
                keep.append(sorted_segs[i])
        
        # 如果过滤后太少，返回原始列表（避免全部过滤）
        if len(keep) < 2 and n >= 2:
            return sorted_segs
        
        return keep

    # ================================================================
    # 中线计算
    # ================================================================

    def compute_guidance_point(self, segments):
        """
        计算巡线引导点
        
        奇数条线段：取中间那条的中心x
        偶数条线段：取中间两条的中心点
        
        Args:
            segments: 过滤后的线段列表
            
        Returns:
            guidance_x: 引导点x坐标（ROI坐标系），None表示无有效线段
            num_lines: 有效线段数量
        """
        n = len(segments)
        if n == 0:
            return None, 0
        
        cx_list = sorted([seg['center'][0] for seg in segments])
        
        if n % 2 == 1:
            # 奇数条：取中间
            mid_idx = n // 2
            guidance_x = cx_list[mid_idx]
        else:
            # 偶数条：取中间两条的中点
            left_idx = n // 2 - 1
            right_idx = n // 2
            guidance_x = (cx_list[left_idx] + cx_list[right_idx]) // 2
        
        return guidance_x, n

    # ================================================================
    # 黑色边界线检测
    # ================================================================

    def detect_black_border(self, roi, roi_w, roi_h):
        """
        检测黑色边界线（禁区边界）
        
        算法：V < mean_v - 25（暗区域），竖向形态学操作增强竖线
        
        Args:
            roi: ROI区域图像
            roi_w, roi_h: ROI宽高
            
        Returns:
            black_lines: 检测到的黑线列表
        """
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        
        mean_v = np.mean(v_ch)
        black_thresh = int(mean_v - 25)
        black_thresh = max(20, min(60, black_thresh))
        
        # 暗区域（黑色边界线）
        _, black_mask = cv2.threshold(v_ch, black_thresh, 255, cv2.THRESH_BINARY_INV)
        
        # 低饱和度过滤
        sat_mask = cv2.inRange(s_ch, 0, 100)
        black_mask = cv2.bitwise_and(black_mask, sat_mask)
        
        # 竖向形态学操作（增强竖直黑线）
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, self.kernel_black)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, self.kernel_open)
        black_mask = cv2.dilate(black_mask, self.kernel_dilate, iterations=1)
        
        # 查找轮廓
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        black_lines = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 80:  # 最小面积阈值
                continue
            
            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = float(max(bw, bh)) / max(min(bw, bh), 1)
            if aspect_ratio < 0.5 or aspect_ratio > 30.0:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                black_lines.append({
                    'center': (cx, cy),
                    'area': area,
                    'bbox': (x, y, bw, bh),
                    'contour': cnt
                })
        
        return black_lines

    # ================================================================
    # 避障力计算
    # ================================================================

    def compute_avoidance(self, black_lines, roi_w):
        """
        计算避障角速度 - 分层避障策略
        
        V31关键修复：
        - 黑线在左(cx < center) -> side = -1 -> 右转远离(angular.z < 0)
        - 黑线在右(cx > center) -> side = +1 -> 左转远离(angular.z > 0)
        
        分层策略：
        - 安全区 (>120px): 轻微预警
        - 危险区 (80px): 猛刹车
        - 预警区 (45px): 直接停 + 猛转
        - 极近距离: 最大力
        
        Args:
            black_lines: 检测到的黑线列表
            roi_w: ROI宽度
            
        Returns:
            avoid_angular: 避障角速度
            nearest_dist: 最近黑线距离
            nearest_side: 最近黑线方向（-1=左, +1=右, 0=无）
        """
        if not black_lines:
            return 0.0, 999, 0
        
        center_x = roi_w // 2
        
        # 找最近的黑线
        nearest = None
        nearest_dist = 9999
        for bl in black_lines:
            cx = bl['center'][0]
            dist = abs(cx - center_x)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest = bl
        
        if nearest is None:
            return 0.0, 999, 0
        
        cx = nearest['center'][0]
        dist = abs(cx - center_x)
        
        # V31修正：黑线在左->side=-1（右转远离），黑线在右->side=+1（左转远离）
        side = -1 if cx < center_x else 1
        
        # 安全区：轻微预警（只有单边时）
        if dist > self.black_safe_distance:
            if len(black_lines) == 1:
                return 0.05 * (-side), dist, side
            return 0.0, dist, side
        
        # 预警区（<45px）：直接最大力
        if dist < self.black_warn_distance:
            force = self.avoid_max * (1.3 if len(black_lines) == 1 else 1.0)
        # 危险区（<80px）：大力度
        elif dist < self.black_danger_distance:
            force = self.avoid_max * (1.15 if len(black_lines) == 1 else 1.0)
        # 安全区与危险区之间：线性插值
        else:
            ratio = 1.0 - float(dist - self.black_danger_distance) / \
                    (self.black_safe_distance - self.black_danger_distance)
            force = self.avoid_max * ratio
        
        force = min(force, self.avoid_max)
        avoid_angular = force * side
        
        return avoid_angular, dist, side

    # ================================================================
    # 锥形桶检测与避障
    # ================================================================

    def detect_and_avoid_cones(self, roi, roi_w, roi_h):
        """
        检测红色锥形桶并计算避障角速度
        
        使用HSV双范围红色检测（红色在HSV中跨越0度/180度边界）
        
        Args:
            roi: ROI区域图像
            roi_w, roi_h: ROI宽高
            
        Returns:
            cone_angular: 锥形桶避障角速度（0表示未检测到）
        """
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 红色在HSV中跨越0度，需要两个范围
        mask1 = cv2.inRange(hsv, self.cone_red_lower, self.cone_red_upper)
        mask2 = cv2.inRange(hsv, self.cone_red_lower2, self.cone_red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center_x = roi_w // 2
        nearest_cone = None
        nearest_dist = 9999
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.cone_min_area:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                dist = abs(cx - center_x)
                
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_cone = (cx, cy, area)
        
        if nearest_cone is None:
            self.cone_detected = False
            return 0.0
        
        self.cone_detected = True
        self.cone_cx, self.cone_cy, cone_area = nearest_cone
        
        # 锥形桶避障逻辑
        cone_side = -1 if self.cone_cx < center_x else 1
        
        if nearest_dist < self.cone_avoid_distance:
            # 距离近：大幅转向
            return self.cone_avoid_angular * cone_side
        else:
            # 距离远：轻微转向
            return 0.1 * self.cone_avoid_angular * cone_side

    # ================================================================
    # 调试图像构建
    # ================================================================

    def build_debug_image(self, roi, roi_w, roi_h, center_x,
                          segments, guidance_x, num_lines,
                          black_lines, nearest_black_dist, nearest_black_side,
                          twist):
        """
        构建调试图像，用于可视化巡线状态
        
        可视化元素：
        - 绿色中线参考线
        - 白色圆点 = 有效白线段
        - 灰色X = 被过滤的孤立线段
        - 红色方框 = 黑色禁区边界
        - 大红色矩形 = 危险区（+/-80px）
        - 半透明红色填充 = 预警区（+/-45px）
        - 黄色矩形外框 = 安全区（+/-120px）
        - 蓝色圆点 = 锥形桶中心
        """
        debug = roi.copy()
        
        # 中心参考线
        cv2.line(debug, (center_x, 0), (center_x, roi_h), (0, 255, 0), 2)
        
        # 画黑线
        for i, bl in enumerate(black_lines):
            cx, cy = bl['center']
            x, y, bw, bh = bl['bbox']
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(debug, "B%d" % (i + 1), (cx - 10, cy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 危险区大矩形（+/-80px）
        danger_left = max(0, center_x - self.black_danger_distance)
        danger_right = min(roi_w, center_x + self.black_danger_distance)
        cv2.rectangle(debug, (danger_left, 0), (danger_right, roi_h), (0, 0, 255), 2)
        
        # 预警区半透明填充（+/-45px）
        warn_left = max(0, center_x - self.black_warn_distance)
        warn_right = min(roi_w, center_x + self.black_warn_distance)
        overlay = debug.copy()
        cv2.rectangle(overlay, (warn_left, 0), (warn_right, roi_h), (0, 0, 255), -1)
        cv2.addWeighted(overlay, 0.12, debug, 0.88, 0, debug)
        
        # 安全区外框（+/-120px）
        safe_left = max(0, center_x - self.black_safe_distance)
        safe_right = min(roi_w, center_x + self.black_safe_distance)
        cv2.rectangle(debug, (safe_left, 0), (safe_right, roi_h), (0, 255, 255), 1)
        
        # 画白线段中心点
        for i, seg in enumerate(segments):
            cx, cy = seg['center']
            cv2.circle(debug, (cx, cy), 6, (255, 255, 255), -1)
            cv2.putText(debug, str(i + 1), (cx - 5, cy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 引导线
        if guidance_x is not None:
            cv2.line(debug, (guidance_x, 0), (guidance_x, roi_h), (0, 255, 255), 4)
            cv2.circle(debug, (guidance_x, roi_h // 2), 12, (0, 255, 0), 2)
        
        # 冲出警告
        if self.outbound_recover_frames > 0:
            cv2.putText(debug, "!!! OUTBOUND RECOVER !!!",
                       (roi_w // 4, roi_h // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        
        # 单边预警
        if len(black_lines) == 1:
            cv2.putText(debug, "SINGLE BORDER!", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        # 信息文字
        state_color = {
            "TRACKING": (0, 255, 0),
            "LOST": (0, 165, 255),
            "OUTBOUND": (0, 0, 255),
            "CONE_AVOID": (255, 0, 255)
        }.get(self.current_state, (128, 128, 128))
        
        y_offset = 25
        texts = [
            ("State: %s" % self.current_state, state_color),
            ("Lines: %d" % num_lines, (255, 255, 0)),
            ("Spd: %.2f | Ang: %.3f" % (twist.linear.x, twist.angular.z), (0, 255, 255)),
            ("Avoid: dist=%d side=%s" % (nearest_black_dist,
                 "L" if nearest_black_side < 0 else "R" if nearest_black_side > 0 else "N"), 
             (0, 0, 255) if nearest_black_dist < self.black_safe_distance else (128, 128, 128)),
            ("Cone: %s" % ("YES" if self.cone_detected else "NO"), 
             (255, 0, 255) if self.cone_detected else (128, 128, 128)),
        ]
        
        for text, color in texts:
            cv2.putText(debug, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_offset += 20
        
        return debug

    # ================================================================
    # 主循环
    # ================================================================

    def run(self):
        """
        主循环 - 保持节点运行
        """
        rospy.loginfo("[LineTracker] Entering main loop.")
        rospy.spin()

    def shutdown(self):
        """
        关闭回调 - 停止机器人
        """
        rospy.loginfo("[LineTracker] Shutdown requested. Stopping robot...")
        self.cmd_vel_pub.publish(Twist())


# ====================================================================
# 入口点
# ====================================================================

def main():
    try:
        tracker = LineTracker()
        rospy.on_shutdown(tracker.shutdown)
        tracker.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
