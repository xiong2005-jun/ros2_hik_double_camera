# ros2_hik_double_camera
本项目基于君佬的海康相机包二次开发，源码参考：https://github.com/chenjunnn/ros2_hik_camera
支持同时驱动两台海康威视工业相机

## 启动方法
source install/setup.bash

ros2 launch hik_camera hik_camera.launch.py

## 使用方法
在 yaml 配置文件中修改以下参数：
camera0_serial: "DA7093654"
    相机序列号（未知可留空,之后可查看终端信息）
camera0_info_url: "package://hik_camera/config/camera0.yaml"
camera0_exposure: 6000
    相机曝光值
camera0_gain: 0.0
    相机增益值

## 相机标定流程（基于陈君的相机包）
1. 安装标定功能包
sudo apt install ros-humble-camera-calibration

source install/setup.bash

2. 启动相机驱动
ros2 launch hik_camera hik_camera.launch.py

3. 新开终端启动标定节点
ros2 run camera_calibration cameracalibrator --size 6x9 --square 0.024 image:=/image_raw camera:=/camera


#rviz2启动
可能出现No Image的问题
1. 点击 Image 条目左边的小箭头 ▶，展开所有子选项
2. 往下滚动，找到 「Topic」 选项，点击右侧的下拉框
       → 选择你的相机话题：/camera_0/image_raw（或 /camera_1/image_raw）
3. 继续往下滚动，就能看到 「QoS」 这个大选项，再点它左边的 ▶ 展开
展开 QoS 后，你会看到 4 个选项，只改这两个：
    Reliability（可靠性）：
        默认是 Default / Reliable
        [点击下拉框，选择 Best Effort]
    Durability（持久性）：
        默认是 Default / Transient Local
        [点击下拉框，选择 Volatile]
之后就能正常显示图像了。

#注意事项
1. 两个相机必须插在不同的 USB3.0 控制器（不同组口）
2. 不要插在拓展坞上，单相机会出现极大延迟，双相机可能跑不起来。
