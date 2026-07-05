#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
inspection_task.py - 巡检任务整合模块

按顺序识别4个区域（A/B/C/D）的仪表盘状态
每次识别后调用语音播报
全部完成后发布 /inspection_complete 话题

Author: Contest Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic
"""
from __future__ import print_function, division
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge

try:
    from common.msg import MeterResult, InspectionResult
except ImportError:
    rospy.logwarn("[InspectionTask] common.msg not available")

from meter_reader import MeterReader
from letter_ocr import LetterOCR


class InspectionTask(object):
    """
    巡检任务整合类
    
    管理4个区域的巡检流程，每个区域依次进行字母识别和仪表盘读数。
    """

    def __init__(self):
        rospy.init_node("inspection_task", anonymous=False)
        rospy.loginfo("[InspectionTask] Initializing inspection task node...")

        # 子模块
        self.meter_reader = MeterReader()
        self.letter_ocr = LetterOCR()
        self.bridge = CvBridge()

        # 状态
        self.current_area_idx = 0
        self.areas = ['A', 'B', 'C', 'D']
        self.results = []
        self.is_running = False
        self.current_image = None

        # ROS接口
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1
        )
        self.result_pub = rospy.Publisher("/meter_result", MeterResult, queue_size=10)
        self.complete_pub = rospy.Publisher("/inspection_complete", InspectionResult, queue_size=1)
        self.voice_pub = rospy.Publisher("/voice_request", String, queue_size=1)

        # 定时器（2Hz巡检处理）
        self.timer = rospy.Timer(rospy.Duration(0.5), self.process_timer)

        rospy.loginfo("[InspectionTask] Node ready.")

    def image_callback(self, msg):
        """相机图像回调"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("[InspectionTask] Image callback error: %s", str(e))

    def start_inspection(self):
        """开始巡检"""
        self.current_area_idx = 0
        self.results = []
        self.is_running = True
        rospy.loginfo("[InspectionTask] Inspection started.")
        self.speak("开始巡检识别")

    def process_timer(self, event):
        """定时处理巡检任务"""
        if not self.is_running:
            return
        
        if self.current_image is None:
            return
        
        if self.current_area_idx >= len(self.areas):
            # 全部完成
            self.is_running = False
            self.publish_complete()
            return
        
        area = self.areas[self.current_area_idx]
        
        # 识别仪表盘
        status, confidence, debug = self.meter_reader.read_meter(self.current_image)
        
        if status is not None:
            # 创建结果消息
            result = MeterResult()
            result.area_letter = area
            result.meter_status = status
            result.confidence = confidence
            
            self.results.append(result)
            self.result_pub.publish(result)
            
            # 播报
            if status in ["偏低", "偏高"]:
                self.speak("%s区域仪表盘显示%s，状态异常" % (area, status))
            else:
                self.speak("%s区域仪表盘显示%s" % (area, status))
            
            rospy.loginfo("[InspectionTask] Area %s: %s (%.2f)", area, status, confidence)
            
            self.current_area_idx += 1
        else:
            rospy.logwarn_throttle(2.0, "[InspectionTask] Failed to read meter for area %s", area)

    def publish_complete(self):
        """发布巡检完成消息"""
        complete_msg = InspectionResult()
        complete_msg.results = self.results
        complete_msg.all_complete = True
        self.complete_pub.publish(complete_msg)
        
        rospy.loginfo("[InspectionTask] Inspection complete. %d results.", len(self.results))
        self.speak("巡检识别全部完成")

    def speak(self, text):
        """发送语音播报请求"""
        msg = String()
        msg.data = text
        self.voice_pub.publish(msg)

    def run(self):
        rospy.spin()

    def shutdown(self):
        pass


def main():
    try:
        task = InspectionTask()
        rospy.on_shutdown(task.shutdown)
        task.start_inspection()
        task.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
