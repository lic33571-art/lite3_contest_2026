#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
contest_fsm.py - 比赛主控制器（有限状态机）

管理2026中国高校智能机器人创意大赛（四足大型组）完整任务流程：
INIT -> LINE_TRACKING -> INSPECTION -> NAVIGATE_TO_GRASP -> GRASP_TASK -> COMPLETE

Author: Robot Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic
"""
from __future__ import print_function, division, absolute_import

import os
import sys
import time
import math
import collections
import threading

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

# 导入自定义消息
try:
    from common.msg import MeterResult, InspectionResult, TaskStatus
except ImportError:
    rospy.logwarn("[ContestFSM] common.msg not found, using std_msgs fallback")
    # 如果common包未编译，使用简单的替代方案
    class MeterResult:
        def __init__(self):
            self.area_letter = ""
            self.meter_status = ""
            self.confidence = 0.0
    
    class InspectionResult:
        def __init__(self):
            self.results = []
            self.all_complete = False
    
    class TaskStatus:
        def __init__(self):
            self.current_task = ""
            self.current_state = ""
            self.progress = 0


class ContestFSM(object):
    """
    比赛有限状态机主控制器
    
    状态转换：
    INIT -> LINE_TRACKING: 机器狗起立完成
    LINE_TRACKING -> INSPECTION: 到达检测区
    INSPECTION -> NAVIGATE_TO_GRASP: 4次识别+播报完成
    NAVIGATE_TO_GRASP -> GRASP_TASK: 到达抓取区
    GRASP_TASK -> COMPLETE: 2次抓取+放置完成
    """

    # ============================================================
    # 状态常量
    # ============================================================
    STATE_INIT = "INIT"
    STATE_LINE_TRACKING = "LINE_TRACKING"
    STATE_INSPECTION = "INSPECTION"
    STATE_NAVIGATE_TO_GRASP = "NAVIGATE_TO_GRASP"
    STATE_GRASP_TASK = "GRASP_TASK"
    STATE_COMPLETE = "COMPLETE"

    def __init__(self):
        rospy.init_node("contest_fsm", anonymous=False)
        rospy.loginfo("[ContestFSM] ==========================================")
        rospy.loginfo("[ContestFSM]  Contest FSM Node Initializing...")
        rospy.loginfo("[ContestFSM]  2026 China University Robot Contest")
        rospy.loginfo("[ContestFSM]  Quadruped Large Group")
        rospy.loginfo("[ContestFSM] ==========================================")

        # ============================================================
        # 参数加载
        # ============================================================
        self.inspection_zone_distance = rospy.get_param("~inspection_zone_distance", 2.0)
        self.grasp_zone_distance = rospy.get_param("~grasp_zone_distance", 1.5)
        self.max_grasp_attempts = rospy.get_param("~max_grasp_attempts", 3)
        self.max_place_attempts = rospy.get_param("~max_place_attempts", 2)
        self.inspection_timeout = rospy.get_param("~inspection_timeout", 120.0)

        rospy.loginfo("[ContestFSM] Parameters loaded:")
        rospy.loginfo("  inspection_zone_distance: %.1f m", self.inspection_zone_distance)
        rospy.loginfo("  grasp_zone_distance: %.1f m", self.grasp_zone_distance)
        rospy.loginfo("  max_grasp_attempts: %d", self.max_grasp_attempts)
        rospy.loginfo("  max_place_attempts: %d", self.max_place_attempts)

        # ============================================================
        # 状态变量
        # ============================================================
        self.current_state = self.STATE_INIT
        self.previous_state = None
        self.state_entry_time = rospy.Time.now()
        
        # 巡检结果存储
        self.inspection_results = []  # MeterResult列表
        self.inspection_complete = False
        self.abnormal_areas = []  # 异常区域字母列表
        
        # 抓取任务状态
        self.grasp_count = 0        # 已完成抓取次数
        self.max_grasp_count = 2    # 需要抓取的总次数
        self.current_target_area = None  # 当前放置目标区域
        
        # 里程计位置
        self.current_position = None
        self.start_position = None
        
        # 线程锁
        self.state_lock = threading.Lock()
        self.data_lock = threading.Lock()
        
        # 机械臂状态
        self.arm_status = "IDLE"
        self.arm_command_pending = False

        # ============================================================
        # ROS通信接口
        # ============================================================
        
        # ---- 发布 ----
        # 速度指令（转发给运动主机）
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 播报请求
        self.voice_pub = rospy.Publisher("/voice_request", String, queue_size=1)
        
        # 机械臂命令
        self.arm_cmd_pub = rospy.Publisher("/arm_command", String, queue_size=1)
        
        # 任务状态
        self.task_status_pub = rospy.Publisher("/task_status", TaskStatus, queue_size=1)
        
        # ---- 订阅 ----
        # 巡检结果
        self.meter_result_sub = rospy.Subscriber(
            "/meter_result", MeterResult, self.meter_result_callback, queue_size=10
        )
        
        # 巡检完成通知
        self.inspection_complete_sub = rospy.Subscriber(
            "/inspection_complete", InspectionResult, self.inspection_complete_callback, queue_size=1
        )
        
        # 机械臂状态
        self.arm_status_sub = rospy.Subscriber(
            "/arm_status", String, self.arm_status_callback, queue_size=1
        )
        
        # 腿部里程计
        self.odom_sub = rospy.Subscriber(
            "/leg_odom", Odometry, self.odom_callback, queue_size=1
        )
        
        # IMU数据
        self.imu_sub = rospy.Subscriber(
            "/imu/data", Imu, self.imu_callback, queue_size=1
        )

        # 定时器（10Hz状态机更新）
        self.timer = rospy.Timer(rospy.Duration(0.1), self.state_machine_update)
        
        rospy.loginfo("[ContestFSM] ROS interfaces initialized.")
        rospy.loginfo("[ContestFSM] Node ready. State: %s", self.current_state)

    # ================================================================
    # 状态转换
    # ================================================================

    def transition_to(self, new_state):
        """
        状态转换
        
        记录状态变化，执行退出/进入回调
        
        Args:
            new_state: 目标状态字符串
        """
        with self.state_lock:
            if self.current_state == new_state:
                return
            
            old_state = self.current_state
            self.previous_state = old_state
            self.current_state = new_state
            self.state_entry_time = rospy.Time.now()
            
            rospy.loginfo("[ContestFSM] ========== STATE TRANSITION ==========")
            rospy.loginfo("[ContestFSM]  %s  -->  %s", old_state, new_state)
            rospy.loginfo("[ContestFSM] ====================================")
            
            # 状态进入回调
            self.on_state_enter(new_state, old_state)

    def on_state_enter(self, new_state, old_state):
        """
        状态进入回调
        
        每个状态进入时执行的初始化操作
        """
        if new_state == self.STATE_INIT:
            rospy.loginfo("[ContestFSM] Entering INIT state. Waiting for robot to stand up.")
            self.speak("系统初始化完成，等待起立")
            
        elif new_state == self.STATE_LINE_TRACKING:
            rospy.loginfo("[ContestFSM] Entering LINE_TRACKING state. Starting line following.")
            self.speak("开始巡线导航")
            # 记录起始位置（用于判断距离）
            if self.current_position is not None:
                self.start_position = self.current_position.copy()
                
        elif new_state == self.STATE_INSPECTION:
            rospy.loginfo("[ContestFSM] Entering INSPECTION state. Stopping for meter reading.")
            # 停止运动
            self.cmd_vel_pub.publish(Twist())
            self.speak("到达检测区，开始巡检识别")
            # 清空之前的巡检结果
            with self.data_lock:
                self.inspection_results = []
                self.inspection_complete = False
                self.abnormal_areas = []
                
        elif new_state == self.STATE_NAVIGATE_TO_GRASP:
            rospy.loginfo("[ContestFSM] Entering NAVIGATE_TO_GRASP state.")
            self.speak("巡检完成，前往抓取区")
            # 记录导航起始位置
            if self.current_position is not None:
                self.start_position = self.current_position.copy()
                
        elif new_state == self.STATE_GRASP_TASK:
            rospy.loginfo("[ContestFSM] Entering GRASP_TASK state. Starting grasp operations.")
            self.speak("到达抓取区，开始抓取任务")
            with self.data_lock:
                self.grasp_count = 0
            
        elif new_state == self.STATE_COMPLETE:
            rospy.loginfo("[ContestFSM] Entering COMPLETE state. All tasks finished!")
            # 停止运动
            self.cmd_vel_pub.publish(Twist())
            self.speak("所有任务已完成")
            
            # 打印最终成绩报告
            self.print_final_report()

    # ================================================================
    # 状态机主更新循环
    # ================================================================

    def state_machine_update(self, event):
        """
        状态机主更新循环（10Hz）
        
        根据当前状态执行对应的行为逻辑
        """
        with self.state_lock:
            state = self.current_state
        
        # 发布任务状态
        self.publish_task_status()
        
        # 状态分发
        if state == self.STATE_INIT:
            self.handle_init_state()
        elif state == self.STATE_LINE_TRACKING:
            self.handle_line_tracking_state()
        elif state == self.STATE_INSPECTION:
            self.handle_inspection_state()
        elif state == self.STATE_NAVIGATE_TO_GRASP:
            self.handle_navigate_to_grasp_state()
        elif state == self.STATE_GRASP_TASK:
            self.handle_grasp_task_state()
        elif state == self.STATE_COMPLETE:
            self.handle_complete_state()

    # ================================================================
    # 各状态处理函数
    # ================================================================

    def handle_init_state(self):
        """
        INIT状态处理
        
        等待机器狗起立完成，然后切换到LINE_TRACKING
        实际比赛中可以通过检测姿态或直接延迟来触发
        """
        elapsed = (rospy.Time.now() - self.state_entry_time).to_sec()
        
        # 简化：等待3秒后自动进入巡线（实际可改为检测机器狗姿态）
        if elapsed > 3.0:
            rospy.loginfo("[ContestFSM] INIT complete. Transitioning to LINE_TRACKING.")
            self.transition_to(self.STATE_LINE_TRACKING)

    def handle_line_tracking_state(self):
        """
        LINE_TRACKING状态处理
        
        巡线导航状态：
        - 在LINE_TRACKING状态下，速度指令由line_tracker节点发布到/cmd_vel
        - 主控制器只需监控状态，不需要额外干预
        - 当到达检测区时，切换到INSPECTION
        
        检测区判断方法（可配置）：
        1. 视觉检测：检测到配电柜/变压器特征
        2. 里程计距离：行走距离达到检测区距离
        3. 时间/帧数：预设时间
        """
        # 方法2：基于里程计距离判断
        if self.current_position is not None and self.start_position is not None:
            distance = np.linalg.norm(self.current_position - self.start_position)
            
            if distance >= self.inspection_zone_distance:
                rospy.loginfo("[ContestFSM] Reached inspection zone (distance: %.2f m)", distance)
                self.transition_to(self.STATE_INSPECTION)

    def handle_inspection_state(self):
        """
        INSPECTION状态处理
        
        巡检识别状态：
        - 机器狗已停止（在进入时发送了零速指令）
        - 等待meter_recognition完成4次识别
        - 收到/inspection_complete后切换到NAVIGATE_TO_GRASP
        - 带超时保护
        """
        elapsed = (rospy.Time.now() - self.state_entry_time).to_sec()
        
        with self.data_lock:
            complete = self.inspection_complete
            results = list(self.inspection_results)
        
        # 检查是否完成
        if complete:
            rospy.loginfo("[ContestFSM] Inspection complete. %d results received.", len(results))
            
            # 提取异常区域
            self.abnormal_areas = []
            for r in results:
                if r.meter_status in ["偏低", "偏高"]:
                    self.abnormal_areas.append(r.area_letter)
            
            rospy.loginfo("[ContestFSM] Abnormal areas: %s", str(self.abnormal_areas))
            
            # 播报汇总
            if self.abnormal_areas:
                self.speak("巡检完成，异常区域为" + "、".join(self.abnormal_areas))
            else:
                self.speak("巡检完成，所有区域正常")
            
            self.transition_to(self.STATE_NAVIGATE_TO_GRASP)
            return
        
        # 超时保护
        if elapsed > self.inspection_timeout:
            rospy.logwarn("[ContestFSM] Inspection timeout! Proceeding with available results.")
            
            with self.data_lock:
                self.inspection_complete = True
            
            self.speak("巡检超时，使用已有结果")
            self.transition_to(self.STATE_NAVIGATE_TO_GRASP)

    def handle_navigate_to_grasp_state(self):
        """
        NAVIGATE_TO_GRASP状态处理
        
        导航到抓取区：
        - 继续沿白线前进
        - 到达抓取区后切换到GRASP_TASK
        - 基于里程计距离判断
        """
        # 方法：基于里程计距离判断
        if self.current_position is not None and self.start_position is not None:
            distance = np.linalg.norm(self.current_position - self.start_position)
            
            if distance >= self.grasp_zone_distance:
                rospy.loginfo("[ContestFSM] Reached grasp zone (distance: %.2f m)", distance)
                self.transition_to(self.STATE_GRASP_TASK)

    def handle_grasp_task_state(self):
        """
        GRASP_TASK状态处理
        
        抓取任务状态：
        - 控制机械臂执行抓取和放置
        - 执行2次抓取+放置
        - 完成后切换到COMPLETE
        """
        with self.data_lock:
            g_count = self.grasp_count
            abnormal = list(self.abnormal_areas)
        
        if g_count >= self.max_grasp_count:
            rospy.loginfo("[ContestFSM] All grasp tasks complete.")
            self.transition_to(self.STATE_COMPLETE)
            return
        
        # 确定当前抓取目标
        if g_count < len(abnormal):
            target_area = abnormal[g_count]
        else:
            target_area = "A"  # 默认
        
        self.current_target_area = target_area
        
        # 执行一次抓取+放置
        success = self.execute_grasp_cycle(target_area)
        
        if success:
            with self.data_lock:
                self.grasp_count += 1
            rospy.loginfo("[ContestFSM] Grasp cycle %d/%d complete.", 
                         self.grasp_count, self.max_grasp_count)
        else:
            rospy.logwarn("[ContestFSM] Grasp cycle failed. Retrying...")

    def handle_complete_state(self):
        """
        COMPLETE状态处理
        
        任务完成状态：
        - 停止所有运动
        - 保持当前状态等待比赛结束
        """
        # 持续发布零速确保停止
        self.cmd_vel_pub.publish(Twist())

    # ================================================================
    # 抓取执行
    # ================================================================

    def execute_grasp_cycle(self, target_area):
        """
        执行一次完整的抓取+放置周期
        
        流程：
        1. 发送GRASP_RED命令
        2. 等待抓取完成
        3. 发送PLACE_TO_X命令
        4. 等待放置完成
        
        Args:
            target_area: 目标放置区域（"A"/"B"/"C"/"D"）
            
        Returns:
            bool: 是否成功
        """
        rospy.loginfo("[ContestFSM] Executing grasp cycle for area %s", target_area)
        
        # 步骤1: 抓取红色长条
        rospy.loginfo("[ContestFSM] Step 1: Grasping red block...")
        self.send_arm_command("GRASP_RED")
        
        if not self.wait_for_arm_status("GRASP_OK", timeout=15.0):
            rospy.logwarn("[ContestFSM] Grasp failed!")
            # 尝试回位
            self.send_arm_command("HOME")
            rospy.sleep(2.0)
            return False
        
        rospy.loginfo("[ContestFSM] Grasp successful!")
        self.speak("抓取成功")
        
        # 步骤2: 放置到目标区域
        place_cmd = "PLACE_TO_" + target_area
        rospy.loginfo("[ContestFSM] Step 2: Placing to %s...", target_area)
        self.send_arm_command(place_cmd)
        
        if not self.wait_for_arm_status("PLACE_OK", timeout=15.0):
            rospy.logwarn("[ContestFSM] Place failed!")
            self.send_arm_command("HOME")
            rospy.sleep(2.0)
            return False
        
        rospy.loginfo("[ContestFSM] Place successful!")
        self.speak("放置到" + target_area + "区域完成")
        
        # 步骤3: 机械臂归位
        self.send_arm_command("HOME")
        rospy.sleep(1.0)
        
        return True

    def send_arm_command(self, command):
        """
        发送机械臂命令
        
        Args:
            command: 命令字符串
        """
        msg = String()
        msg.data = command
        self.arm_cmd_pub.publish(msg)
        rospy.loginfo("[ContestFSM] Arm command sent: %s", command)

    def wait_for_arm_status(self, expected_status, timeout=15.0):
        """
        等待机械臂达到期望状态
        
        Args:
            expected_status: 期望的状态字符串
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否在超时前达到期望状态
        """
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.arm_status == expected_status:
                return True
            if self.arm_status.endswith("FAIL"):
                return False
            rate.sleep()
        
        rospy.logwarn("[ContestFSM] Timeout waiting for arm status: %s", expected_status)
        return False

    # ================================================================
    # 语音播报
    # ================================================================

    def speak(self, text):
        """
        发送语音播报请求
        
        Args:
            text: 要播报的中文文本
        """
        msg = String()
        msg.data = text
        self.voice_pub.publish(msg)
        rospy.loginfo("[ContestFSM] Voice request: %s", text)

    # ================================================================
    # 回调函数
    # ================================================================

    def meter_result_callback(self, msg):
        """
        仪表盘识别结果回调
        
        在INSPECTION状态下收集识别结果
        """
        with self.data_lock:
            self.inspection_results.append(msg)
        rospy.loginfo("[ContestFSM] Meter result: Area=%s, Status=%s, Conf=%.2f",
                     msg.area_letter, msg.meter_status, msg.confidence)

    def inspection_complete_callback(self, msg):
        """
        巡检完成回调
        
        收到后标记巡检完成
        """
        rospy.loginfo("[ContestFSM] Inspection complete notification received.")
        with self.data_lock:
            self.inspection_complete = True
            if msg.results:
                self.inspection_results = list(msg.results)

    def arm_status_callback(self, msg):
        """
        机械臂状态回调
        """
        self.arm_status = msg.data
        rospy.logdebug("[ContestFSM] Arm status: %s", msg.data)

    def odom_callback(self, msg):
        """
        里程计回调
        
        记录当前位置用于距离判断
        """
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])

    def imu_callback(self, msg):
        """
        IMU回调
        
        可用于检测机器狗姿态（是否起立等）
        """
        pass  # 目前未使用，可扩展

    # ================================================================
    # 任务状态发布
    # ================================================================

    def publish_task_status(self):
        """
        发布当前任务状态
        """
        try:
            status = TaskStatus()
            status.current_task = self.current_state
            
            # 计算进度
            progress_map = {
                self.STATE_INIT: 0,
                self.STATE_LINE_TRACKING: 10,
                self.STATE_INSPECTION: 30,
                self.STATE_NAVIGATE_TO_GRASP: 50,
                self.STATE_GRASP_TASK: 70,
                self.STATE_COMPLETE: 100
            }
            base_progress = progress_map.get(self.current_state, 0)
            
            # 在GRASP_TASK状态下根据完成次数微调
            if self.current_state == self.STATE_GRASP_TASK:
                with self.data_lock:
                    g_count = self.grasp_count
                base_progress += int((g_count / float(self.max_grasp_count)) * 25)
            
            status.progress = min(base_progress, 100)
            status.current_state = "Running"
            
            self.task_status_pub.publish(status)
        except Exception as e:
            rospy.logdebug("[ContestFSM] Failed to publish task status: %s", str(e))

    # ================================================================
    # 最终报告
    # ================================================================

    def print_final_report(self):
        """
        打印最终比赛报告
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("           CONTEST FINAL REPORT")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Total inspection results: %d", len(self.inspection_results))
        
        for r in self.inspection_results:
            rospy.loginfo("  Area %s: %s (confidence: %.2f)",
                         r.area_letter, r.meter_status, r.confidence)
        
        rospy.loginfo("Abnormal areas: %s", str(self.abnormal_areas))
        rospy.loginfo("Grasp count: %d/%d", self.grasp_count, self.max_grasp_count)
        rospy.loginfo("Final state: %s", self.current_state)
        rospy.loginfo("=" * 60)

    # ================================================================
    # 主循环
    # ================================================================

    def run(self):
        """
        主循环
        """
        rospy.loginfo("[ContestFSM] Entering main loop.")
        rospy.spin()

    def shutdown(self):
        """
        关闭回调
        """
        rospy.loginfo("[ContestFSM] Shutdown requested. Stopping robot...")
        self.cmd_vel_pub.publish(Twist())


# ====================================================================
# 入口点
# ====================================================================

def main():
    try:
        fsm = ContestFSM()
        rospy.on_shutdown(fsm.shutdown)
        fsm.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
