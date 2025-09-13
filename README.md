# Cyberdog Face Recognition Control

该项目是哈尔滨工业大学计算机实践课程的团队实验项目。  
项目基于人脸识别控制 Cyberdog，在仿真平台中使用摄像头识别小组成员人脸，并触发对应的个性化动作。

## 推荐安装环境
Ubuntu 20.04 + ROS2 Humble + Python 3.8

## 仿真平台
本项目依赖 Cyberdog 仿真平台，请按照以下文档完成安装：  
👉 [Cyberdog 仿真平台文档](https://miroboticslab.github.io/blogs/#/cn/cyberdog_gazebo_cn)

## 数据采集

### 开启摄像头话题
启动ROS2环境
```
source /opt/ros/galactic/setup.bash
```
发布摄像头图像话题
```
ros2 run image_tools cam2image
```

查看摄像头图像
要查看摄像头捕获的图像，可以使用rqt工具：

打开新终端，输入：rqt

在弹出的界面中选择：Plugins → Visualization → Image View

在Image View下方的复选框中选择/image话题

即可看到摄像头捕获的实时图像


### 样本采集
```
python3 src/face_data_collection.py
```
程序会依次提示收集指定人员的人脸数据，每人收集50张样本图像。数据会保存到 face_data/姓名/

训练人脸识别模型
```
python3 train_face_recognition.py
```
使用收集到的人脸数据训练SVM模型，并保存为models/face_recognition_model.pkl


### 启动仿真平台
终端1 - 启动gazebo仿真环境
```
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_gazebo gazebo.launch.py
```

终端2 - 启动控制程序
```
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

终端3 - 启动可视化界面
```
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_visual cyberdog_visual.launch.py
```

### 运行人脸识别与机器狗控制
```
python3 src/face.py
```

### 自定义人员动作
修改face.py文件中的person_specific_action函数，为不同人员配置专属动作：
以A同学 - 握手为例
```
def person_specific_action(ctrl, msg, person_name):
    if person_name == "A同学":
        # 配置握手动作参数
        msg.mode = 62
        msg.gait_id = 1
        msg.vel_des = [0, 0, 0] 
        msg.duration = 8700
        msg.step_height = [0.05, 0.05]
```




### 运行人脸识别与控制
```
python3 src/cyberdog_simulator/colortest2/face.py
```

### 效果

摄像头识别到人脸后，程序会输出姓名与置信度

Cyberdog 根据识别结果执行动作：

A同学 → 握手



## 🎬 演示方法

1. 启动 Cyberdog 仿真平台
```
cd ~/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```
打开摄像头话题，发布图像流


```
ros2 run image_tools cam2image
```
使用 rqt 工具查看图像话题

```
rqt
```
在插件中选择 Image View，订阅 /image 话题。

启动人脸识别与控制程序

```
python3 src/cyberdog_simulator/colortest2/face.py
```

当摄像头检测到人脸后，程序会输出识别结果（姓名 + 置信度）。

根据不同成员触发 Cyberdog 执行动作：
A同学 → 握手

B同学 → 太空步

C同学 → 扭屁股

D同学 → 跳起


## 声明
本项目仅用于课程实验和学习，不涉及商业用途。
