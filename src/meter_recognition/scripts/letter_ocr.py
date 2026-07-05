#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
letter_ocr.py - 区域字母OCR识别模块

使用OpenCV模板匹配识别A/B/C/D字母
字母规格：A4尺寸(297x210mm)，Arial字体，200磅
贴在检测区配电柜/变压器两侧

Author: Contest Team
Version: 1.0
Compatible: Python 2/3, ROS Noetic, OpenCV 4.x
"""
from __future__ import print_function, division
import cv2
import numpy as np
import os


class LetterOCR(object):
    """
    区域字母OCR识别器
    
    双模式识别：
    1. 模板匹配（默认）：生成A/B/C/D模板，归一化相关系数匹配
    2. Tesseract OCR（备选）：PSM 10单字符模式
    """

    SUPPORTED_LETTERS = ['A', 'B', 'C', 'D']

    def __init__(self):
        """初始化OCR识别器"""
        try:
            import rospy
            self._loginfo = rospy.loginfo
            self._logwarn = rospy.logwarn
        except ImportError:
            self._loginfo = lambda msg: print("[INFO] " + str(msg))
            self._logwarn = lambda msg: print("[WARN] " + str(msg))
        
        self.template_width = 100
        self.template_height = 140
        self.template_min_confidence = 0.3
        
        self._templates = {}
        self._load_templates()
        self._loginfo("LetterOCR initialized")

    def _load_templates(self):
        """生成字母模板图像"""
        for letter in self.SUPPORTED_LETTERS:
            template = np.zeros((self.template_height, self.template_width), dtype=np.uint8)
            # 使用OpenCV绘制字母作为模板
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size = cv2.getTextSize(letter, font, 3.5, 6)[0]
            text_x = (self.template_width - text_size[0]) // 2
            text_y = (self.template_height + text_size[1]) // 2
            cv2.putText(template, letter, (text_x, text_y), font, 3.5, 255, 6)
            self._templates[letter] = template
        self._loginfo("Templates generated for: " + str(self.SUPPORTED_LETTERS))

    def recognize(self, image):
        """
        识别图像中的字母
        
        Args:
            image: BGR格式图像
            
        Returns:
            (letter, confidence): 识别结果和置信度
        """
        if image is None or image.size == 0:
            return None, 0.0
        
        # 预处理
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        
        # 二值化
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0.0
        
        # 找到最大的轮廓（假设是字母）
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # 提取字母区域
        letter_roi = gray[y:y+h, x:x+w]
        
        # 缩放到模板大小
        letter_resized = cv2.resize(letter_roi, (self.template_width, self.template_height))
        _, letter_binary = cv2.threshold(letter_resized, 127, 255, cv2.THRESH_BINARY_INV)
        
        # 模板匹配
        best_letter = None
        best_score = -1.0
        
        for letter, template in self._templates.items():
            result = cv2.matchTemplate(letter_binary, template, cv2.TM_CCOEFF_NORMED)
            score = np.max(result)
            if score > best_score:
                best_score = score
                best_letter = letter
        
        confidence = float(best_score)
        
        if confidence < self.template_min_confidence:
            return None, confidence
        
        return best_letter, confidence

    def recognize_in_image(self, image, roi=None):
        """
        在图像的指定区域中识别字母
        
        Args:
            image: BGR格式图像
            roi: 感兴趣区域 (x, y, w, h)，None表示处理整张图像
            
        Returns:
            (letter, confidence, bbox): 识别结果、置信度和边界框
        """
        if roi is not None:
            x, y, w, h = roi
            h_img, w_img = image.shape[:2]
            x = max(0, x)
            y = max(0, y)
            w = min(w, w_img - x)
            h = min(h, h_img - y)
            roi_image = image[y:y+h, x:x+w]
        else:
            roi_image = image
            x, y = 0, 0
        
        letter, confidence = self.recognize(roi_image)
        
        # 计算字母在原始图像中的边界框
        if letter is not None:
            gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY) if len(roi_image.shape) == 3 else roi_image
            _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                bx, by, bw, bh = cv2.boundingRect(largest)
                bbox = (x + bx, y + by, bw, bh)
            else:
                bbox = (x, y, roi_image.shape[1], roi_image.shape[0])
        else:
            bbox = None
        
        return letter, confidence, bbox

    def detect_and_recognize(self, image):
        """
        在图像中自动检测字母区域并识别
        
        流程：
        1. 图像预处理（灰度化、二值化）
        2. 轮廓检测
        3. 筛选符合字母特征的轮廓
        4. 对每个候选区域进行模板匹配
        5. 返回最佳匹配结果
        
        Args:
            image: BGR格式图像
            
        Returns:
            results: 列表，每个元素为 (letter, confidence, bbox)
        """
        if image is None:
            return []
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()
        
        # 自适应阈值二值化
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
        
        # 形态学操作去噪
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        results = []
        h_img, w_img = gray.shape[:2]
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # 面积过滤
            if area < 500 or area > w_img * h_img * 0.5:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            
            # 宽高比过滤（字母通常有一定宽高比）
            aspect_ratio = float(w) / max(h, 1)
            if aspect_ratio < 0.2 or aspect_ratio > 2.0:
                continue
            
            # 提取候选区域
            padding = 10
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(w_img, x + w + padding)
            y2 = min(h_img, y + h + padding)
            
            candidate = gray[y1:y2, x1:x2]
            
            # 缩放到模板大小进行匹配
            candidate_resized = cv2.resize(candidate, (self.template_width, self.template_height))
            _, candidate_binary = cv2.threshold(candidate_resized, 127, 255, cv2.THRESH_BINARY_INV)
            
            # 与所有模板匹配
            best_letter = None
            best_score = -1.0
            
            for letter, template in self._templates.items():
                result = cv2.matchTemplate(candidate_binary, template, cv2.TM_CCOEFF_NORMED)
                score = np.max(result)
                if score > best_score:
                    best_score = score
                    best_letter = letter
            
            confidence = float(best_score)
            
            if confidence >= self.template_min_confidence and best_letter is not None:
                results.append((best_letter, confidence, (x, y, w, h)))
        
        # 按置信度排序
        results.sort(key=lambda r: r[1], reverse=True)
        
        return results

    def draw_results(self, image, results):
        """
        在图像上绘制识别结果
        
        Args:
            image: BGR格式图像
            results: 识别结果列表 [(letter, confidence, bbox), ...]
            
        Returns:
            debug_image: 绘制了结果的图像
        """
        debug = image.copy()
        
        for letter, confidence, bbox in results:
            x, y, w, h = bbox
            # 画框
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 画文字
            text = "%s (%.2f)" % (letter, confidence)
            cv2.putText(debug, text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug


def main():
    """测试函数"""
    import rospy
    rospy.init_node("letter_ocr_test")
    
    ocr = LetterOCR()
    
    # 测试用：创建包含字母的测试图像
    test_image = np.ones((200, 200, 3), dtype=np.uint8) * 255
    cv2.putText(test_image, "A", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 0), 10)
    
    results = ocr.detect_and_recognize(test_image)
    print("Detection results:", results)


if __name__ == "__main__":
    main()
