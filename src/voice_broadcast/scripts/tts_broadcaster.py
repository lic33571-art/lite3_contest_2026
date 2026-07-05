#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tts_broadcaster.py - 中文语音播报模块

支持 pyttsx3 / edge-tts / espeak 三引擎自动切换
播报格式："A区域仪表盘显示偏低，状态异常"

Author: Contest Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic
"""
from __future__ import print_function, division

import os
import sys
import subprocess
import tempfile
import threading
import queue

import rospy
from std_msgs.msg import String


class TTSBroadcaster(object):
    """
    语音播报器
    
    三引擎自动切换：
    1. pyttsx3（离线，首选）
    2. edge-tts（在线备选）
    3. espeak（系统命令备选）
    
    支持播报队列管理，避免重叠播报。
    """

    def __init__(self):
        rospy.init_node("tts_broadcaster", anonymous=False)
        rospy.loginfo("[TTS] Initializing TTS broadcaster...")

        # 参数
        self.tts_engine = rospy.get_param("~tts_engine", "pyttsx3")
        self.language = rospy.get_param("~language", "zh-CN")
        self.rate = rospy.get_param("~rate", 150)
        self.volume = rospy.get_param("~volume", 1.0)

        # 播报队列
        self.speech_queue = queue.Queue()
        self.is_speaking = False
        self.current_engine = None

        # 初始化TTS引擎
        self._init_engine()

        # ROS接口
        self.voice_sub = rospy.Subscriber(
            "/voice_request", String, self.voice_callback, queue_size=10
        )
        self.complete_pub = rospy.Publisher("~broadcast_complete", String, queue_size=1)

        # 启动播报线程
        self.speech_thread = threading.Thread(target=self._speech_loop)
        self.speech_thread.daemon = True
        self.speech_thread.start()

        rospy.loginfo("[TTS] TTS broadcaster ready. Engine: %s", self.current_engine)

    def _init_engine(self):
        """初始化TTS引擎"""
        # 尝试pyttsx3
        if self.tts_engine == "pyttsx3":
            try:
                import pyttsx3
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', self.rate)
                self.engine.setProperty('volume', self.volume)
                self.current_engine = "pyttsx3"
                rospy.loginfo("[TTS] pyttsx3 engine initialized")
                return
            except Exception as e:
                rospy.logwarn("[TTS] pyttsx3 failed: %s", str(e))

        # 尝试edge-tts
        try:
            import edge_tts
            self.current_engine = "edge-tts"
            rospy.loginfo("[TTS] edge-tts engine initialized")
            return
        except ImportError:
            rospy.logwarn("[TTS] edge-tts not available")

        # 回退到espeak
        self.current_engine = "espeak"
        rospy.loginfo("[TTS] Using espeak fallback")

    def voice_callback(self, msg):
        """
        语音请求回调
        
        收到播报请求后添加到队列
        """
        text = msg.data
        rospy.loginfo("[TTS] Received voice request: %s", text)
        self.speech_queue.put(text)

    def _speech_loop(self):
        """
        播报主循环
        
        从队列中取出文本并播报
        """
        while not rospy.is_shutdown():
            try:
                text = self.speech_queue.get(timeout=1.0)
                self.is_speaking = True
                self._speak(text)
                self.is_speaking = False
                
                # 发布播报完成通知
                complete_msg = String()
                complete_msg.data = text
                self.complete_pub.publish(complete_msg)
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr("[TTS] Speech error: %s", str(e))
                self.is_speaking = False

    def _speak(self, text):
        """
        执行播报
        
        根据当前引擎选择对应的播报方式
        """
        rospy.loginfo("[TTS] Speaking: %s", text)
        
        if self.current_engine == "pyttsx3":
            self._speak_pyttsx3(text)
        elif self.current_engine == "edge-tts":
            self._speak_edge_tts(text)
        elif self.current_engine == "espeak":
            self._speak_espeak(text)

    def _speak_pyttsx3(self, text):
        """使用pyttsx3播报"""
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.setProperty('rate', self.rate)
            engine.setProperty('volume', self.volume)
            engine.say(text)
            engine.runAndWait()
        except Exception as e:
            rospy.logwarn("[TTS] pyttsx3 error: %s, falling back", str(e))
            self._speak_espeak(text)

    def _speak_edge_tts(self, text):
        """使用edge-tts播报"""
        try:
            import edge_tts
            import asyncio
            
            communicate = edge_tts.Communicate(text, voice="zh-CN-XiaoxiaoNeural")
            
            # 生成临时音频文件
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp:
                tmp_path = tmp.name
            
            communicate.save_sync(tmp_path)
            
            # 播放音频
            subprocess.call(['mpg123', '-q', tmp_path])
            
            # 清理
            os.unlink(tmp_path)
            
        except Exception as e:
            rospy.logwarn("[TTS] edge-tts error: %s, falling back", str(e))
            self._speak_espeak(text)

    def _speak_espeak(self, text):
        """使用espeak播报"""
        try:
            cmd = ['espeak', '-v', 'zh', '-s', str(self.rate), text]
            subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            rospy.logerr("[TTS] espeak error: %s", str(e))
            rospy.loginfo("[TTS] Text (not spoken): %s", text)

    def run(self):
        rospy.spin()

    def shutdown(self):
        pass


def main():
    try:
        broadcaster = TTSBroadcaster()
        rospy.on_shutdown(broadcaster.shutdown)
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
