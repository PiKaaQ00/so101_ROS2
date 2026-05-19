<div align="center">
  <h1>so101_ros2</h1>
  <p>SO101 follower arm ROS2 read-only integration without lerobot.</p>
  <p>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"></a>
    <a href="https://www.python.org/"><img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version"></a>
    <a href="https://docs.ros.org/en/humble/Installation.html"><img src="https://img.shields.io/badge/ROS2-Humble-green.svg" alt="ROS2 Version"></a>
  </p>
</div>

---

## 项目状态

本仓库当前用于标准 SO101 从臂的 ROS2 只读连接测试。

已经完成：

```text
不依赖 lerobot
直接连接 SO101 从臂 Feetech STS3215 舵机
检测 1-6 号舵机是否在线
读取每个舵机当前位置
发布 ROS2 joint_states
在 RViz 中显示机械臂模型
支持零位和方向配置
中文测试日志
```

当前节点是低风险只读测试节点，不会向机械臂发送运动命令。

---

## 已完成的功能

### 1. 不依赖 lerobot

新增节点直接通过串口与 SO101 从臂舵机通信，不需要安装或导入 `lerobot`。

节点文件：

```text
so101_ros2_bridge/so101_ros2_bridge/direct_follower_readonly_node.py
```

### 2. ROS2 关节状态发布

节点会发布：

```text
/follower/joint_states
/follower/joint_states_raw
```

查看一次关节状态：

```bash
ros2 topic echo /follower/joint_states --once
```

连续查看：

```bash
ros2 topic echo /follower/joint_states
```

### 3. RViz 模型显示

启动只读测试时可以打开 RViz。RViz 中的 SO101 模型会根据 `/follower/joint_states` 更新姿态。

---

## 当前不会做的事情

当前版本不会控制机械臂运动。

不会执行：

```text
不发送舵机目标位置
不订阅 /follower/joint_commands
不启动 arm_controller
不启动 gripper_controller
不执行轨迹控制
不执行遥操作
```

---

## 主要新增文件

只读测试节点：

```text
so101_ros2_bridge/so101_ros2_bridge/direct_follower_readonly_node.py
```

只读测试配置：

```text
so101_ros2_bridge/config/so101_direct_follower_readonly.yaml
```

只读测试启动文件：

```text
so101_bringup/launch/so101_readonly_test.launch.py
```

Python 入口已添加到：

```text
so101_ros2_bridge/setup.py
```

依赖声明已添加到：

```text
so101_ros2_bridge/package.xml
```

---

## 依赖

需要 ROS2 Humble。

需要串口 Python 包：

```bash
sudo apt install python3-serial
```

---

## 编译

在 ROS2 工作空间根目录执行：

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 启动只读连接测试

不启动 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=false
```

启动 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=true
```

根据实际情况修改串口：

```text
/dev/ttyACM0
/dev/ttyACM1
```

正常连接时会看到类似日志：

```text
准备打开从臂串口：/dev/ttyACM0，波特率：1000000。
串口已打开，开始检测 1-6 号舵机。
shoulder_pan：检测到 1 号舵机。
shoulder_lift：检测到 2 号舵机。
elbow_flex：检测到 3 号舵机。
wrist_flex：检测到 4 号舵机。
wrist_roll：检测到 5 号舵机。
gripper：检测到 6 号舵机。
所有舵机均已响应。
只读连接测试节点已启动。该节点不会发送运动目标。
```

---

## 零位和方向配置

配置文件：

```text
so101_ros2_bridge/config/so101_direct_follower_readonly.yaml
```

核心参数：

```yaml
signs: [1, 1, 1, 1, 1, 1]
zero_positions: [2076, 1957, 2056, 2241, 1920, 2050]
```

关节顺序：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

`zero_positions` 表示 ROS 关节角为 0 时对应的舵机原始位置。

`signs` 表示方向：

```text
1  表示同向
-1 表示反向
```

换算公式：

```text
ros_angle = sign * (raw_position - zero_position) / 4096.0 * 2π
```

---

## 修改零位

1. 将真实机械臂摆到参考姿态。

2. 启动只读测试：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=false
```

3. 查看日志中的舵机原始位置：

```text
当前舵机原始位置：[2076, 816, 3086, 2893, 2083, 2050]
```

4. 如果希望当前真实姿态对应 RViz 的关节零位，直接写入：

```yaml
zero_positions: [2076, 816, 3086, 2893, 2083, 2050]
```

5. 如果希望当前真实姿态对应仓库默认折叠姿态，需要根据目标角度反算 `zero_positions`。

默认折叠姿态文件：

```text
so101_description/config/initial_positions.yaml
```

默认角度：

```yaml
shoulder_pan: 0.0
shoulder_lift: -1.75
elbow_flex: 1.58
wrist_flex: 1.0
wrist_roll: 0.25
gripper: 0.0
```

反算公式：

```text
zero_position = raw_position - target_angle / (2π) * 4096.0 / sign
```

修改后重新编译并 source：

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 修改方向

如果真实机械臂某个关节往一个方向动，但 RViz 中该关节往反方向动，修改对应的 `signs`。

例如 `elbow_flex` 是第 3 个关节，如果它方向反了：

```yaml
signs: [1, 1, -1, 1, 1, 1]
```

---

## 常见问题

### RViz 模型在两个姿态之间快速切换

通常是 `/follower/joint_states` 有多个发布者。

检查：

```bash
ros2 topic info /follower/joint_states -v
```

正常只读测试应为：

```text
Publisher count: 1
```

如果有多个发布者，清理旧进程：

```bash
pkill -f direct_follower_readonly_node
pkill -f robot_state_publisher
pkill -f rviz2
```

然后重新启动。

### 串口打不开

检查串口和权限：

```bash
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
```

如果在虚拟机中使用真机，还需要确认 USB 已直通到虚拟机。

---

## 后续计划

当前只完成只读连接、状态发布和 RViz 显示。

后续如果要实现运动控制，应先增加单关节、小角度、带限位检查的安全写入测试，再接入完整 ROS2 controller。

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the full license text.
