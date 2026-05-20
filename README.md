# so101_ros2

SO101 从臂 ROS2 工程。当前版本面向标准 SO101 follower arm，使用 Feetech STS3215 舵机，重点实现不依赖 `lerobot` 的 ROS2 连接、RViz 同步和 MoveIt 规划执行。

当前代码以 C++ 驱动为主，launch 文件使用 Python。

## 当前已完成功能

- 不依赖 `lerobot`，可以直接通过串口连接 SO101 从臂。
- C++ 只读节点可以检测 1-6 号舵机并读取当前位置。
- 发布 `/follower/joint_states`，RViz 中的模型可以和真实机械臂同步。
- 支持单关节低风险写入测试。
- 已创建 `so101_moveit_config`，可以在 MoveIt 中规划。
- 已实现 `FollowJointTrajectory` action server，MoveIt 的 `Execute` 可以发送轨迹到真机。
- 支持执行端速度限制、插值执行和基础抖动过滤参数。

## 硬件假设

- 机械臂：标准 SO101 从臂
- 舵机：Feetech STS3215
- 舵机 ID：默认 1-6
- 串口示例：`/dev/ttyACM0`
- ROS2：Humble

关节顺序固定为：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

## 主要新增内容

```text
so101_direct_driver/
so101_moveit_config/
RVIZ_REAL_ARM_SYNC.md
README_DIRECT_FOLLOWER_READONLY.md
so101_bringup/launch/so101_readonly_test.launch.py
so101_bringup/launch/so101_single_joint_test.launch.py
so101_bringup/launch/so101_moveit_trajectory_check.launch.py
so101_bringup/launch/so101_trajectory_action_server.launch.py
```

## 编译

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

如果串口没有权限：

```bash
sudo usermod -aG dialout $USER
```

然后注销重新登录，或临时执行：

```bash
sudo chmod 666 /dev/ttyACM0
```

## 只读连接测试

这个节点只读舵机位置，不会发送运动目标，适合第一次连接测试。

不打开 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py \
  port:=/dev/ttyACM0 \
  display:=false
```

打开 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py \
  port:=/dev/ttyACM0 \
  display:=true
```

查看关节状态：

```bash
ros2 topic echo /follower/joint_states --once
```

查看原始舵机位置：

```bash
ros2 topic echo /follower/joint_states_raw --once
```

## RViz 和真实机械臂同步原理

同步链路是：

```text
STS3215 舵机原始位置
  -> so101_direct_driver C++ 节点
  -> /follower/joint_states
  -> robot_state_publisher
  -> /tf
  -> RViz RobotModel
```

我们不需要自己手写 TF。动态 TF 由 `robot_state_publisher` 根据 URDF 和 `/follower/joint_states` 自动发布。

详细说明见：

```text
RVIZ_REAL_ARM_SYNC.md
```

## 零位和方向配置

配置文件：

```text
so101_ros2_bridge/config/so101_direct_follower_readonly.yaml
```

核心参数：

```yaml
motor_ids: [1, 2, 3, 4, 5, 6]
zero_positions: [2076, 1957, 2056, 2241, 1920, 2050]
signs: [1, 1, 1, 1, 1, 1]
range_mins: [851, 811, 887, 987, 0, 2025]
range_maxs: [3438, 3215, 3090, 3219, 4095, 3493]
```

换算公式：

```text
ros_angle = sign * (raw_position - zero_position) / 4096.0 * 2π
```

如果 RViz 方向和真实机械臂方向相反，修改对应关节的 `signs`。

如果 RViz 初始位姿和真实机械臂不一致，修改 `zero_positions`。

## 单关节低风险写入测试

第一次写入真机前，建议先测试单个关节的小幅运动。

示例：测试夹爪：

```bash
ros2 launch so101_bringup so101_single_joint_test.launch.py \
  port:=/dev/ttyACM0 \
  joint:=gripper \
  delta_rad:=0.02 \
  enable_write:=true \
  display:=false
```

示例：测试腕部旋转：

```bash
ros2 launch so101_bringup so101_single_joint_test.launch.py \
  port:=/dev/ttyACM0 \
  joint:=wrist_roll \
  delta_rad:=0.05 \
  enable_write:=true \
  display:=false
```

参数说明：

```text
joint          要测试的关节名
delta_rad      相对当前位置变化量，单位 rad
enable_write   true 才会真正写入舵机
```

## MoveIt 规划

只启动 MoveIt，用于规划和 RViz 交互：

```bash
ros2 launch so101_moveit_config moveit_planning.launch.py
```

连接真实机械臂状态：

```bash
ros2 launch so101_moveit_config moveit_planning.launch.py \
  use_fake_joint_states:=false \
  joint_states_topic:=/follower/joint_states
```

当前 IK 使用 `pick_ik`，适合 SO101 这类自由度不足以严格满足完整 6D 位姿约束的机械臂。

MoveIt 中已经添加 arm 预设位姿：

```text
folded
home
ready
pick_pre
place_left
place_right
calib_front
```

其中 `home` 和 `folded` 都表示回到初始折叠位姿。

这些预设位姿只是在 MoveIt 中作为目标状态使用。选择预设后，需要先 `Plan`，确认轨迹安全，再点击 `Execute` 才会控制真机。

安装：

```bash
sudo apt update
sudo apt install ros-humble-pick-ik
```

## MoveIt 真机执行

真机执行需要两个终端。

终端 1，启动 C++ 轨迹执行 action server：

```bash
ros2 launch so101_bringup so101_trajectory_action_server.launch.py \
  port:=/dev/ttyACM0 \
  time_scale:=1.0 \
  max_velocity_rad_s:=0.06 \
  max_step_rad:=0.01 \
  command_rate:=15.0 \
  command_deadband_ticks:=3
```

终端 2，启动 MoveIt：

```bash
ros2 launch so101_moveit_config moveit_planning.launch.py \
  use_fake_joint_states:=false \
  joint_states_topic:=/follower/joint_states \
  allow_execute:=true
```

确认 action server 存在：

```bash
ros2 action list | grep follow_joint_trajectory
```

应该看到：

```text
/follower_arm_controller/follow_joint_trajectory
```

## 真机执行参数

```text
max_velocity_rad_s       最大关节速度，越小越慢
max_step_rad             插值最大步长，越小越细
command_rate             轨迹执行时目标发送频率
command_deadband_ticks   目标 raw tick 死区，过滤很小的重复写入
time_scale               对 MoveIt 轨迹时间做整体缩放
```

推荐稳定测试参数：

```bash
max_velocity_rad_s:=0.06
max_step_rad:=0.01
command_rate:=15.0
command_deadband_ticks:=3
```

如果仍然抖动，可以更保守：

```bash
max_velocity_rad_s:=0.04
max_step_rad:=0.01
command_rate:=10.0
command_deadband_ticks:=4
```

当前执行端属于初版位置控制。要进一步做到更快更稳，后续应使用 STS3215 舵机内部速度/加速度控制，或实现多舵机同步写入。

## USB 相机

安装 USB 相机驱动：

```bash
sudo apt install ros-humble-usb-cam
```

腕部 USB 相机相关代码放在 `so101_vision` 包中。`camera_link` 默认挂在 `follower/gripper_link` 下，TF 由 C++ 节点发布，同时会发布标准 ROS 光学坐标系 `camera_optical_frame`。

启动腕部 USB 相机和 `camera_link`：

```bash
ros2 launch so101_vision so101_wrist_camera.launch.py \
  video_device:=/dev/video0
```

默认图像话题：

```text
/camera/image_raw
/camera/camera_info
```

查看图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

查看相机 TF：

```bash
ros2 run tf2_ros tf2_echo follower/gripper_link camera_link
```

查看光学坐标系 TF：

```bash
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
```

图像消息的 `frame_id` 使用：

```text
camera_optical_frame
```

如果需要修改腕部相机的粗略外参：

```bash
ros2 launch so101_vision so101_wrist_camera.launch.py \
  video_device:=/dev/video0 \
  parent_frame:=follower/gripper_link \
  camera_x:=-0.005 \
  camera_y:=0.032 \
  camera_z:=-0.020 \
  camera_roll:=0.0 \
  camera_pitch:=0.0 \
  camera_yaw:=1.5708
```

这些外参只是初始估计。后续做手眼标定后，需要用标定结果替换 `camera_x/y/z/roll/pitch/yaw`。

## 安全注意事项

- 第一次测试必须低速、小幅度。
- 真机执行前，手和线缆离开机械臂运动范围。
- 不要同时启动 `so101_readonly_test.launch.py` 和 `so101_trajectory_action_server.launch.py`，它们会抢同一个串口。
- 如果 Execute 失败，先看 action server 终端日志。
- 如果 RViz 模型来回跳，检查 `/follower/joint_states` 是否有多个发布者：

```bash
ros2 topic info /follower/joint_states -v
```

## 当前限制

- MoveIt 真机执行已经可用，但轨迹平滑性仍受串口写入方式和舵机内部控制影响。
- 当前 action server 主要控制 5 个 arm 关节，夹爪 MoveIt 控制后续再完善。
- 暂未实现 STS3215 内部速度寄存器控制。
- 暂未实现多舵机同步写入。

## 后续计划

- 添加夹爪 MoveIt 控制。
- 添加常用预设位姿，例如 `home`、`ready`、`folded`。
- 使用 STS3215 内部速度参数改善执行平滑度。
- 实现多舵机同步写入。
- 增加手眼标定辅助节点。

## License

MIT License.
