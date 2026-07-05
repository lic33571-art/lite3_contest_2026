# 2026中国高校智能机器人创意大赛 — 四足大型组

绝影Lite3四足机器人比赛完整解决方案 | ROS Noetic | NVIDIA Jetson Xavier NX

---

## 项目简介

本项目为2026年中国高校智能机器人创意大赛（四足大型组）国赛的完整ROS集成框架，基于**绝影Lite3**四足机器人和**NVIDIA Jetson Xavier NX**计算平台开发。

**三大比赛任务**：
1. **巡线导航** — 白线段检测跟踪 + 黑色边界线避障 + 锥形桶避障
2. **巡检识别** — 仪表盘指针识别（正常/偏低/偏高）+ 区域字母OCR（A/B/C/D）
3. **长条抓取** — 颜色识别 + 机械臂抓取与放置

**评分占比**：线下挑战60%（避障10分 + 巡检40分 + 抓取50分）+ 技术报告40%

### 关键信息

| 项目 | 规格 |
|------|------|
| 四足机器人 | 绝影Lite3系列 |
| AI主机 | NVIDIA Jetson Xavier NX |
| 深度相机 | Intel RealSense D435i |
| 激光雷达 | **不允许使用** |
| 场地尺寸 | 5000mm x 6000mm |
| 路径引导 | 六条均匀白线段（国赛） |
| 禁区边界 | 黑色边界线 |

---

## 目录结构

```
.
├── README.md                          # 本文档
├── config/
│   └── contest_params.yaml            # 所有模块的共享参数配置
└── src/
    ├── common/                        # 通用消息定义包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── MeterResult.msg        # 仪表盘识别结果
    │       ├── InspectionResult.msg   # 巡检结果（4区域+完成标志）
    │       └── TaskStatus.msg         # 任务状态反馈
    ├── main_controller/               # 主控制器 — 状态机调度
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/
    │   │   └── contest_fsm.py         # 比赛FSM核心
    │   └── launch/
    │       └── contest.launch         # 全系统一键启动
    ├── line_tracking/                 # 巡线导航模块
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/
    │   │   └── line_tracker.py        # 巡线核心（V31优化版）
    │   └── launch/
    │       └── line_tracking.launch   # 巡线单独启动
    ├── meter_recognition/             # 仪表盘巡检识别模块
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/
    │   │   ├── meter_reader.py        # 指针识别
    │   │   ├── letter_ocr.py          # 字母OCR
    │   │   └── inspection_task.py     # 巡检任务整合
    │   └── launch/
    │       └── meter_recognition.launch
    ├── voice_broadcast/               # 语音播报模块
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/
    │   │   └── tts_broadcaster.py     # TTS播报
    │   └── launch/
    │       └── voice_broadcast.launch
    └── arm_control/                   # 机械臂抓取模块
        ├── CMakeLists.txt
        ├── package.xml
        ├── scripts/
        │   ├── arm_controller.py      # 机械臂控制接口
        │   └── grasp_task.py          # 抓取+放置逻辑
        └── launch/
            └── arm_control.launch
```

---

## 依赖安装说明

### 系统环境

- Ubuntu 20.04 LTS (Focal Fossa)
- ROS Noetic Ninjemys
- Python 3.8+

### ROS依赖包安装

```bash
# 基础ROS包
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description

# 消息生成依赖
sudo apt-get install -y \
    ros-noetic-message-generation \
    ros-noetic-message-runtime
```

### Python依赖安装

```bash
# 基础库
pip3 install -U pip
pip3 install numpy opencv-python scipy

# 语音播报
pip3 install pyttsx3 edge-tts

# OCR识别
pip3 install pytesseract
sudo apt-get install -y tesseract-ocr tesseract-ocr-chi-sim

# 机械臂控制（根据实际机械臂型号调整）
pip3 install pyserial

# 其他工具
pip3 install rospkg catkin_pkg
```

### RealSense SDK安装

```bash
# 添加RealSense库
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

---

## 编译和运行方法

### 1. 创建工作空间

```bash
# 克隆代码到工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/lic33571-art/lite3_contest_2026.git

# 安装依赖
rosdep install --from-paths . --ignore-src -r -y
```

### 2. 编译

```bash
cd ~/catkin_ws
catkin_make

# 首次编译时，common包需优先编译以生成消息头文件：
catkin_make --pkg common
catkin_make

# 刷新环境
source devel/setup.bash
```

### 3. 运行

#### 一键启动全部节点

```bash
roslaunch main_controller contest.launch
```

#### 单独启动各模块

```bash
# 巡线导航
roslaunch line_tracking line_tracking.launch

# 巡检识别
roslaunch meter_recognition meter_recognition.launch

# 语音播报
roslaunch voice_broadcast voice_broadcast.launch

# 机械臂控制
roslaunch arm_control arm_control.launch
```

#### 快捷调试命令

```bash
# 查看任务状态
rostopic echo /task_status

# 手动发送播报请求
rostopic pub /voice_request std_msgs/String "data: '测试播报'"

# 查看识别结果
rostopic echo /meter_result

# 查看巡检完成通知
rostopic echo /inspection_complete

# 记录ROS bag
rosbag record -a -O contest_run.bag
```

---

## 各模块说明

### main_controller — 主控制器

基于有限状态机（FSM）的任务调度核心，管理全局任务流程。

| 状态 | 说明 | 转换条件 |
|------|------|----------|
| INIT | 初始化 | 机器狗起立完成 |
| LINE_TRACKING | 巡线导航 | 自主模式开启 |
| INSPECTION | 巡检识别 | 到达检测区 |
| NAVIGATE_TO_GRASP | 导航到抓取区 | 4次识别完成 |
| GRASP_TASK | 抓取放置 | 到达抓取区 |
| COMPLETE | 任务完成 | 2次抓取放置完成 |

### line_tracking — 巡线导航模块

基于V31优化版巡线算法，适配国赛白线段地图。

**核心功能**：
- 白线段检测：HSV色彩空间高亮度阈值 + 形态学操作
- 聚类过滤：去除间距>100px的孤立线段
- 中线计算：奇数条取中间，偶数条取中点
- 黑色边界线避障：分层避障（安全区/危险区/预警区）
- LOST状态：左右扫描恢复
- 冲出保护：从"有黑线"变"无黑线"时回退
- 锥形桶避障：HSV检测红色锥形桶动态避障

**避障方向修正**（V31关键修复）：
- 黑线在左 → 右转远离 (angular.z < 0)
- 黑线在右 → 左转远离 (angular.z > 0)

### meter_recognition — 巡检识别模块

基于OpenCV的指针角度检测 + Tesseract OCR字母识别。

**核心功能**：
- 霍夫圆检测定位仪表盘
- Canny边缘 + 直线检测识别指针角度
- 颜色分区判断：黄(偏低) / 绿(正常) / 红(偏高)
- 模板匹配 / Tesseract OCR识别A/B/C/D区域字母
- 4个区域结果整合发布

### voice_broadcast — 语音播报模块

基于pyttsx3/edge-tts中文TTS引擎。

**播报示例**：
- "A区域仪表盘显示偏低，状态异常"
- "B区域仪表盘显示正常"

### arm_control — 机械臂抓取模块

基于逆运动学 + 视觉伺服的机械臂控制。

**核心功能**：
- 长条颜色检测（红色/绿色HSV阈值）
- 深度相机获取目标3D位置
- 逆运动学求解抓取位姿
- 抓取 + 移动到放置区 + 释放

---

## 比赛任务流程

```
[出发区] --(起立+自主模式)--> [巡线导航]
                                      |
                           到达检测区 |
                                      v
                           [巡检识别] --(4次识别+播报)--> [导航到抓取区]
                                                                    |
                                                         到达抓取区 |
                                                                    v
                                                           [抓取任务]
                                                                    |
                                                      2次抓取放置完成 |
                                                                    v
                                                               [完成]
```

---

## 配置文件说明

所有可调参数集中在 `config/contest_params.yaml` 中，各模块通过 `rospy.get_param()` 读取：

| 参数段 | 说明 |
|--------|------|
| `line_tracking` | 巡线PD参数、避障阈值、扫描参数、白线检测、锥形桶参数 |
| `meter_recognition` | 仪表盘检测半径、颜色分区角度、OCR置信度阈值 |
| `voice_broadcast` | TTS引擎、语速、音量 |
| `arm_control` | 机械臂自由度、抓取高度、夹爪开度、颜色HSV阈值 |
| `main_controller` | 检测区/抓取区距离阈值、最大重试次数 |

---

## 调试与维护

### 常见问题

| 问题 | 解决方案 |
|------|----------|
| 消息编译失败 | 先编译 `common` 包: `catkin_make --pkg common` |
| RealSense无法启动 | 检查USB3.0连接: `rs-enumerate-devices` |
| 白线检测不稳定 | 调整 `white_line_min_area` 和 `cluster_max_gap` |
| TTS无声音 | 检查音频设备: `pactl list sinks` |
| 机械臂运动异常 | 检查串口权限: `sudo chmod 666 /dev/ttyUSB0` |

### 日志查看

```bash
# 实时查看所有节点日志
roslaunch main_controller contest.launch 2>&1 | tee contest.log
```

---

## 技术报告要点参考

### 系统架构设计
- 模块化ROS架构，6个功能包清晰分离
- 基于发布/订阅模型的松耦合通信
- 自定义消息类型实现模块间数据交换
- 有限状态机管理全局任务流程

### 巡线算法创新点
- V31优化版PD控制器，自适应速度调节
- 白线段聚类过滤算法，抗干扰能力强
- 三层避障策略（安全/危险/预警）
- LOST状态扫描恢复 + 冲出保护机制
- 锥形桶HSV色彩检测动态避障

### 巡检识别算法
- 霍夫变换圆形检测定位仪表盘
- 指针角度计算判断状态区间
- 模板匹配 + Tesseract OCR双模式字母识别
- 4区域结果整合与置信度评估

---

MIT License
