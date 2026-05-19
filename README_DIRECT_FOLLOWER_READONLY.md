# SO101 从臂只读连接测试说明

这份说明记录当前仓库为标准 SO101 从臂新增的“低风险只读连接测试”改动。该方案不依赖 `lerobot`，只直接读取 Feetech STS3215 舵机状态，用于验证从臂连接、发布关节状态，以及在 RViz 中显示模型姿态。

当前版本不会向机械臂发送运动目标。

## 改动内容

新增只读测试节点：

```text
so101_ros2_bridge/so101_ros2_bridge/direct_follower_readonly_node.py
```

功能：

```text
打开从臂串口
检测 1-6 号舵机是否响应
读取每个舵机当前位置
发布 /follower/joint_states
发布 /follower/joint_states_raw
打印中文测试日志
```

不会做：

```text
不订阅 /follower/joint_commands
不发布舵机目标位置
不启动 arm_controller
不启动 gripper_controller
不依赖 lerobot
```

新增只读测试配置：

```text
so101_ros2_bridge/config/so101_direct_follower_readonly.yaml
```

当前配置：

```yaml
/**:
  ros__parameters:
    port: "/dev/ttyACM0"
    baudrate: 1000000
    read_rate: 5.0
    present_position_address: 56
    position_ticks_per_turn: 4096.0
    response_timeout: 0.2
    joint_names:
      - shoulder_pan
      - shoulder_lift
      - elbow_flex
      - wrist_flex
      - wrist_roll
      - gripper
    motor_ids: [1, 2, 3, 4, 5, 6]
    homing_offsets: [-1862, 1289, 1062, 1943, 888, 1881]
    range_mins: [851, 811, 887, 987, 0, 2025]
    range_maxs: [3438, 3215, 3090, 3219, 4095, 3493]
    signs: [1, 1, 1, 1, 1, 1]
    zero_positions: [2076, 1957, 2056, 2241, 1920, 2050]
```

新增只读测试 launch：

```text
so101_bringup/launch/so101_readonly_test.launch.py
```

它只启动：

```text
robot_state_publisher
static_transform_publisher
direct_follower_readonly_node
可选 RViz
```

不会启动 ROS2 control 的 controller manager 和控制器。

修改 Python 包入口：

```text
so101_ros2_bridge/setup.py
```

新增入口：

```python
direct_follower_readonly_node = so101_ros2_bridge.direct_follower_readonly_node:main
```

修改依赖声明：

```text
so101_ros2_bridge/package.xml
```

新增：

```xml
<depend>sensor_msgs</depend>
<exec_depend>python3-serial</exec_depend>
```

## 编译

在 ROS2 工作空间根目录执行：

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

如果缺少串口 Python 包：

```bash
sudo apt install python3-serial
```

## 启动只读连接测试

先确认从臂串口，例如 `/dev/ttyACM0`。

启动只读测试，不开 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=false
```

启动只读测试，并打开 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=true
```

正常日志应包含：

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

查看关节状态：

```bash
ros2 topic echo /follower/joint_states --once
```

连续查看：

```bash
ros2 topic echo /follower/joint_states
```

## 零位和方向参数

只读节点把舵机原始位置转换成 ROS 关节角，核心参数是：

```yaml
signs: [1, 1, 1, 1, 1, 1]
zero_positions: [2076, 1957, 2056, 2241, 1920, 2050]
```

关节顺序固定为：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

`zero_positions` 表示每个关节在 ROS 角度为 0 时对应的舵机原始位置。

`signs` 表示方向：

```text
1  表示舵机原始值增大时，ROS 关节角也增大
-1 表示舵机原始值增大时，ROS 关节角减小
```

换算公式：

```text
ros_angle = sign * (raw_position - zero_position) / 4096.0 * 2π
```

## 如何修改零位

1. 先把机械臂手动摆到你希望 RViz 显示匹配的真实姿态。

2. 启动只读测试：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=false
```

3. 查看日志中的原始位置：

```text
当前舵机原始位置：[2076, 816, 3086, 2893, 2083, 2050]
```

4. 如果你希望这个真实姿态在 RViz 中显示为“所有关节角都是 0”，就直接把这 6 个数写入：

```yaml
zero_positions: [2076, 816, 3086, 2893, 2083, 2050]
```

5. 如果你希望这个真实姿态对应仓库默认折叠姿态，需要按默认初始角度反算 `zero_positions`。

仓库默认姿态来自：

```text
so101_description/config/initial_positions.yaml
```

默认值：

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

例如当前原始位置是：

```text
[2076, 816, 3086, 2893, 2083, 2050]
```

希望它对应默认折叠姿态，则当前配置为：

```yaml
zero_positions: [2076, 1957, 2056, 2241, 1920, 2050]
```

6. 修改配置后重新编译并 source：

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

7. 重新启动测试：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py port:=/dev/ttyACM0 display:=true
```

## 如何修改方向

如果真实机械臂某个关节往一个方向动，但 RViz 中该关节往反方向动，就修改对应的 `signs`。

例如 `elbow_flex` 方向反了，它是第 3 个关节：

```yaml
signs: [1, 1, -1, 1, 1, 1]
```

修改后重新编译、source、启动。

## RViz 快速来回跳动

如果 RViz 在两个姿态之间快速切换，优先检查 `/follower/joint_states` 是否有多个发布者：

```bash
ros2 topic info /follower/joint_states -v
```

正常只读测试应为：

```text
Publisher count: 1
```

如果是 2 或更多，说明有旧节点没有关干净。可以先清理：

```bash
pkill -f direct_follower_readonly_node
pkill -f robot_state_publisher
pkill -f rviz2
```

然后重新启动只读测试。

## 安全说明

当前只读测试节点不会让机械臂运动，因为它没有写入舵机目标位置，也没有接入 ROS2 controller。

后续如果要加写入控制，应先做单关节、小角度、限位检查的安全测试，不要直接接完整轨迹控制。
