# RViz 与真实 SO101 从臂同步说明

本文说明当前仓库中“真实机械臂姿态同步到 RViz”的实现方式，以及需要重点查看的代码和配置文件。

当前同步链路是只读链路，不会控制机械臂运动。

## 总体数据流

真实 SO101 从臂到 RViz 的同步流程如下：

```text
Feetech STS3215 舵机
        ↓
C++ 串口读取节点
        ↓
/follower/joint_states
        ↓
robot_state_publisher
        ↓
/tf
        ↓
RViz RobotModel
```

核心思想是：

```text
读取真实舵机原始位置
转换成 ROS 关节角度
发布 sensor_msgs/msg/JointState
robot_state_publisher 根据 URDF 计算每个 link 的 TF
RViz 根据 TF 显示机械臂模型姿态
```

## 关键代码文件

### 1. C++ 只读驱动节点

```text
so101_direct_driver/src/direct_follower_readonly_node.cpp
```

这是现实机械臂和 ROS2 之间的核心同步节点。

主要功能：

```text
打开 /dev/ttyACM*
通过 Feetech 协议 ping 1-6 号舵机
读取每个舵机当前位置
将舵机原始值转换成 ROS 关节角
发布 /follower/joint_states
发布 /follower/joint_states_raw
```

需要重点看这些函数：

```cpp
FeetechReadonlyBus::ping()
FeetechReadonlyBus::read_u16()
DirectFollowerReadonlyNode::publish_positions()
DirectFollowerReadonlyNode::raw_position_to_radians()
```

其中 `publish_positions()` 是同步的核心：

```text
读取 6 个舵机原始位置
检查是否超出标定范围
调用 raw_position_to_radians() 转成弧度
发布 JointState
```

### 2. 只读测试启动文件

```text
so101_bringup/launch/so101_readonly_test.launch.py
```

这个 launch 将同步链路启动起来。

它启动：

```text
robot_state_publisher
static_transform_publisher
so101_direct_driver/direct_follower_readonly_node
可选 RViz
```

其中 C++ 只读节点配置为：

```python
readonly_node = Node(
    package='so101_direct_driver',
    executable='direct_follower_readonly_node',
    name='so101_direct_follower_readonly',
    namespace='follower',
    output='screen',
    parameters=[
        os.path.join(bridge_pkg, 'config', 'so101_direct_follower_readonly.yaml'),
        {'port': port},
    ],
)
```

因为节点在 `namespace='follower'` 下运行，所以它发布的话题是：

```text
/follower/joint_states
/follower/joint_states_raw
```

### 3. 舵机标定和角度转换配置

```text
so101_ros2_bridge/config/so101_direct_follower_readonly.yaml
```

这是 RViz 和现实同步是否准确的关键配置。

当前配置包含：

```yaml
motor_ids: [1, 2, 3, 4, 5, 6]
homing_offsets: [-1862, 1289, 1062, 1943, 888, 1881]
range_mins: [851, 811, 887, 987, 0, 2025]
range_maxs: [3438, 3215, 3090, 3219, 4095, 3493]
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

### 4. URDF 模型

```text
so101_description/urdf/so101_new_calib.urdf.xacro
so101_description/urdf/so101_new_calib.urdf
```

URDF 定义了：

```text
link 结构
joint 名称
joint 连接关系
joint 旋转轴
joint 上下限
mesh 模型
```

RViz 显示的机械臂形状来自 URDF 和 mesh，姿态来自 `/follower/joint_states`。

所以同步必须满足：

```text
JointState 里的关节名
必须和 URDF 里的 joint name 完全一致
```

当前关节名为：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

### 5. robot_state_publisher

启动位置：

```text
so101_description/launch/rsp.launch.py
```

它负责：

```text
读取 robot_description
订阅 joint_states
根据 URDF 计算 TF
发布每个 link 的 /tf
```

在 `so101_readonly_test.launch.py` 中，它以 follower 类型启动，因此使用 follower frame prefix：

```text
follower/base_link
follower/shoulder_link
follower/upper_arm_link
...
```

### 6. RViz 配置

```text
so101_bringup/rviz/robot_display_with_cameras.rviz
```

RViz 中的 RobotModel 使用：

```text

```

它最终根据 `/tf` 中的 follower 坐标系树显示机械臂姿态。

## 原始舵机值如何变成 ROS 关节角

同步的核心公式在：

```text
so101_direct_driver/src/direct_follower_readonly_node.cpp
```

函数：

```cpp
raw_position_to_radians()
```

换算公式：

```text
ros_angle = sign * (raw_position - zero_position) / 4096.0 * 2π
```

含义：

```text
raw_position   舵机当前读取到的原始位置
zero_position  ROS 关节角为 0 时对应的舵机原始位置
sign           方向，1 表示同向，-1 表示反向
4096           STS3215 一圈编码数
2π             一圈对应的弧度
```

如果 RViz 和现实机械臂角度不一致，通常要调整：

```text
zero_positions
signs
```

## 为什么要配置 zero_positions

舵机原始位置不是 ROS 关节角。

例如舵机当前原始值是：

```text
2076
```

但这个值对应 ROS 中的多少弧度，取决于你定义的机械零位。

`zero_positions` 就是在告诉程序：

```text
当某个舵机读到这个原始值时，认为该关节角是 0 rad
```

如果希望真实机械臂某个姿态在 RViz 中显示为默认折叠姿态，就需要根据目标角度反算 `zero_positions`。

## 如何调零位

1. 启动只读节点，不开 RViz 也可以：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py \
  port:=/dev/ttyACM0 \
  display:=false
```

2. 查看日志中的原始位置：

```text
当前舵机原始位置：[2076, 816, 3086, 2893, 2083, 2050]
```

3. 如果希望当前真实姿态对应 RViz 的关节零位，直接写入：

```yaml
zero_positions: [2076, 816, 3086, 2893, 2083, 2050]
```

4. 如果希望当前真实姿态对应某组目标角度，使用公式反算：

```text
zero_position = raw_position - target_angle / (2π) * 4096.0 / sign
```

## 如何调方向

如果真实机械臂某个关节运动方向和 RViz 中相反，就修改 `signs`。

例如 `elbow_flex` 是第 3 个关节，如果方向反了：

```yaml
signs: [1, 1, -1, 1, 1, 1]
```

修改后重新编译并 source：

```bash
cd ~/so_101_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

然后重新启动只读测试。

## 启动同步

启动 RViz 同步显示：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py \
  port:=/dev/ttyACM0 \
  display:=true
```

只启动同步，不打开 RViz：

```bash
ros2 launch so101_bringup so101_readonly_test.launch.py \
  port:=/dev/ttyACM0 \
  display:=false
```

## 验证同步是否正常

查看关节状态：

```bash
ros2 topic echo /follower/joint_states --once
```

查看发布者数量：

```bash
ros2 topic info /follower/joint_states -v
```

正常情况下：

```text
Publisher count: 1
```

如果有多个发布者，RViz 可能会在两个姿态之间来回跳。

清理旧进程：

```bash
pkill -f direct_follower_readonly_node
pkill -f single_joint_write_test_node
pkill -f robot_state_publisher
pkill -f rviz2
```

## 同步链路中每个部分的作用

```text
direct_follower_readonly_node
  读取真实舵机位置，发布 JointState

so101_direct_follower_readonly.yaml
  定义舵机 ID、范围、零位、方向

robot_state_publisher
  根据 URDF 和 JointState 计算 TF

so101_new_calib.urdf.xacro / so101_new_calib.urdf
  定义机械臂模型、关节、连杆、mesh

RViz RobotModel
  根据 robot_description 和 TF 显示模型姿态
```

## 和 MoveIt 的区别

RViz 现实同步只需要：

```text
真实关节状态
URDF
robot_state_publisher
RViz
```

MoveIt 规划还需要：

```text
SRDF
planning group
kinematics.yaml
OMPL 配置
move_group
```

当前 RViz 同步是真机状态显示链路；MoveIt 是规划链路。两者可以使用同一个 URDF，但职责不同。
