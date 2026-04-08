该仓库以君佬的海康相机包为基础改编,详见https://github.com/chenjunnn/ros2_hik_camera
该相机包可同时驱动两个海康相机
####启动方法
ros2 launch hik_camera hik_camera.launch.py

####使用方法
在yaml文件中
    camera0_serial: "DA7093654"
       填入相机序列号，不知道也可以先置空
    camera0_info_url: "package://hik_camera/config/camera0.yaml"
    camera0_exposure: 6000
       这是曝光
    camera0_gain: 0.0
       这是增益
       
####相机标定流程（用陈君的相机包）

# 1. 安装标定工具
sudo apt install ros-humble-camera-calibration

source install/setup.bash

# 2. 打开相机
ros2 launch hik_camera hik_camera.launch.py

# 3. 在新终端中启动标定器（使用正确的参数格式）
ros2 run camera_calibration cameracalibrator --size 6x9 --square 0.024 image:=/image_raw camera:=/camera
# ros2_hik_double_camera
