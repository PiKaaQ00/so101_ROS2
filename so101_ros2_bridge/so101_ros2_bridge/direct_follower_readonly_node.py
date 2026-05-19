#!/usr/bin/env python3

import math
import struct
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:  # pragma: no cover - depends on target system packages
    serial = None


class FeetechReadonlyBus:
    HEADER = b'\xff\xff'
    INST_PING = 0x01
    INST_READ = 0x02

    def __init__(self, port: str, baudrate: int, timeout: float):
        if serial is None:
            raise RuntimeError("没有安装 python3-serial，无法打开串口。")

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def ping(self, motor_id: int) -> bool:
        self._send_packet(motor_id, self.INST_PING, [])
        return self._read_status_packet(expected_id=motor_id) is not None

    def read_u16(self, motor_id: int, address: int) -> int:
        self._send_packet(motor_id, self.INST_READ, [address, 2])
        packet = self._read_status_packet(expected_id=motor_id)
        if packet is None:
            raise TimeoutError(f"读取 {motor_id} 号舵机超时。")

        error, params = packet
        if error:
            raise RuntimeError(f"读取 {motor_id} 号舵机失败，错误码：{error}。")
        if len(params) < 2:
            raise RuntimeError(f"读取 {motor_id} 号舵机失败，返回数据长度不足。")

        return struct.unpack('<H', bytes(params[:2]))[0]

    def _send_packet(self, motor_id: int, instruction: int, params: Iterable[int]):
        params = list(params)
        length = len(params) + 2
        body = [motor_id, length, instruction, *params]
        checksum = (~sum(body)) & 0xFF
        self._serial.write(self.HEADER + bytes(body + [checksum]))
        self._serial.flush()

    def _read_status_packet(self, expected_id: int):
        deadline = time.monotonic() + self._serial.timeout

        while time.monotonic() < deadline:
            byte = self._serial.read(1)
            if byte != b'\xff':
                continue
            if self._serial.read(1) != b'\xff':
                continue

            head = self._serial.read(2)
            if len(head) != 2:
                return None

            motor_id = head[0]
            length = head[1]
            payload = self._serial.read(length)
            if len(payload) != length:
                return None

            error = payload[0]
            params = list(payload[1:-1])
            checksum = payload[-1]
            expected_checksum = (~(motor_id + length + error + sum(params))) & 0xFF
            if checksum != expected_checksum:
                continue
            if motor_id != expected_id:
                continue

            return error, params

        return None


class DirectFollowerReadonlyNode(Node):
    def __init__(self):
        super().__init__('so101_direct_follower_readonly')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('read_rate', 5.0)
        self.declare_parameter('present_position_address', 56)
        self.declare_parameter('position_ticks_per_turn', 4096.0)
        self.declare_parameter('response_timeout', 0.2)
        self.declare_parameter(
            'joint_names',
            [
                'shoulder_pan',
                'shoulder_lift',
                'elbow_flex',
                'wrist_flex',
                'wrist_roll',
                'gripper',
            ],
        )
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('homing_offsets', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('range_mins', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('range_maxs', [4095, 4095, 4095, 4095, 4095, 4095])
        self.declare_parameter('signs', [1, 1, 1, 1, 1, 1])
        self.declare_parameter('zero_positions', [2048, 2048, 2048, 2048, 2048, 2048])

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.position_address = int(self.get_parameter('present_position_address').value)
        self.ticks_per_turn = float(self.get_parameter('position_ticks_per_turn').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.motor_ids = [int(v) for v in self.get_parameter('motor_ids').value]
        self.homing_offsets = [int(v) for v in self.get_parameter('homing_offsets').value]
        self.range_mins = [int(v) for v in self.get_parameter('range_mins').value]
        self.range_maxs = [int(v) for v in self.get_parameter('range_maxs').value]
        self.signs = [int(v) for v in self.get_parameter('signs').value]
        self.zero_positions = [int(v) for v in self.get_parameter('zero_positions').value]
        self._raw_log_count = 0

        self._validate_config()

        timeout = float(self.get_parameter('response_timeout').value)
        self.get_logger().info(f"准备打开从臂串口：{self.port}，波特率：{self.baudrate}。")
        try:
            self.bus = FeetechReadonlyBus(self.port, self.baudrate, timeout)
        except Exception as exc:
            raise RuntimeError(f"打开串口失败：{exc}") from exc
        self.get_logger().info("串口已打开，开始检测 1-6 号舵机。")

        self._ping_motors()

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.raw_joint_state_pub = self.create_publisher(JointState, 'joint_states_raw', 10)

        read_rate = float(self.get_parameter('read_rate').value)
        self.timer = self.create_timer(1.0 / read_rate, self._publish_positions)
        self.get_logger().info("只读连接测试节点已启动。该节点不会发送运动目标。")

    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.close()
            self.get_logger().info("串口已关闭。")
        super().destroy_node()

    def _validate_config(self):
        expected_len = len(self.joint_names)
        fields = {
            'motor_ids': self.motor_ids,
            'homing_offsets': self.homing_offsets,
            'range_mins': self.range_mins,
            'range_maxs': self.range_maxs,
            'signs': self.signs,
            'zero_positions': self.zero_positions,
        }
        for name, values in fields.items():
            if len(values) != expected_len:
                raise ValueError(
                    f"参数 {name} 的数量是 {len(values)}，但关节数量是 {expected_len}。"
                )

    def _ping_motors(self):
        missing = []
        for joint_name, motor_id in zip(self.joint_names, self.motor_ids):
            if self.bus.ping(motor_id):
                self.get_logger().info(f"{joint_name}：检测到 {motor_id} 号舵机。")
            else:
                missing.append((joint_name, motor_id))

        if missing:
            formatted = '，'.join(f"{name}(ID {motor_id})" for name, motor_id in missing)
            raise RuntimeError(f"以下舵机没有响应：{formatted}。")

        self.get_logger().info("所有舵机均已响应。")

    def _publish_positions(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions = []
        raw_positions = []
        for index, (joint_name, motor_id) in enumerate(zip(self.joint_names, self.motor_ids)):
            try:
                raw_position = self.bus.read_u16(motor_id, self.position_address)
            except Exception as exc:
                self.get_logger().error(f"{joint_name}：读取位置失败：{exc}")
                return
            raw_positions.append(raw_position)

            if raw_position < self.range_mins[index] or raw_position > self.range_maxs[index]:
                self.get_logger().warn(
                    f"{joint_name}：当前位置 {raw_position} 超出标定范围 "
                    f"[{self.range_mins[index]}, {self.range_maxs[index]}]。"
                )

            radians = self._raw_position_to_radians(
                raw_position,
                self.zero_positions[index],
                self.signs[index],
            )
            positions.append(radians)

        if self._raw_log_count < 5:
            self.get_logger().info(f"当前舵机原始位置：{raw_positions}")
            self._raw_log_count += 1

        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        self.joint_state_pub.publish(msg)
        self.raw_joint_state_pub.publish(msg)

    def _raw_position_to_radians(self, raw_position: int, zero_position: int, sign: int) -> float:
        return sign * ((raw_position - zero_position) / self.ticks_per_turn) * 2.0 * math.pi


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DirectFollowerReadonlyNode()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"只读连接测试失败：{exc}")
        else:
            print(f"只读连接测试失败：{exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
