#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface.msg import Motor
import sys

class MotorControlTest(Node):
    def __init__(self):
        super().__init__('motor_control_test')
        self.publisher_ = self.create_publisher(Motor, 'motor_cmd', 10)
        self.subscription = self.create_subscription(
            Motor,
            'motor_status',
            self.motor_status_callback,
            10)
        self.get_logger().info('电机控制测试节点已启动')

        # 用于周期性发布的命令缓存
        self._current_mode = None
        self._current_motor_id = None
        self._current_current = None
        self._current_speed = None
        self._current_data = None
        self.timer_ = self.create_timer(0.05, self.periodic_publish)  # 20Hz
    
    def motor_status_callback(self, msg):
        self.get_logger().info(
            f'电机{msg.motor_id}状态: 角度={msg.angle}, 速度={msg.speed}rpm, '
            f'电流={msg.current}, 温度={msg.temperature}°C')
    
    def send_current_command(self, motor_id, current):
        self._current_mode = 'current'
        self._current_motor_id = motor_id
        self._current_current = current
        self._current_speed = None
        self._current_data = None
        self._publish_current_command()

    def _publish_current_command(self):
        if self._current_motor_id is not None and self._current_current is not None:
            msg = Motor()
            msg.mode = Motor.MODE_CURRENT
            msg.motor_id = self._current_motor_id
            msg.current = self._current_current
            self.publisher_.publish(msg)
            self.get_logger().info(f'已发送电流命令: 电机{self._current_motor_id}, 电流={self._current_current}')
    
    def send_speed_command(self, motor_id, speed):
        self._current_mode = 'speed'
        self._current_motor_id = motor_id
        self._current_speed = speed
        self._current_current = None
        self._current_data = None
        self._publish_speed_command()

    def _publish_speed_command(self):
        if self._current_motor_id is not None and self._current_speed is not None:
            msg = Motor()
            msg.mode = Motor.MODE_SPEED
            msg.motor_id = self._current_motor_id
            msg.speed = self._current_speed
            self.publisher_.publish(msg)
            self.get_logger().info(f'已发送速度命令: 电机{self._current_motor_id}, 速度={self._current_speed}rpm')
    
    def send_raw_command(self, data):
        self._current_mode = 'raw'
        self._current_data = data[:8] + [0]*(8-len(data)) if len(data) < 8 else data[:8]
        self._current_motor_id = 0
        self._current_current = None
        self._current_speed = None
        self._publish_raw_command()

    def _publish_raw_command(self):
        if self._current_data is not None:
            msg = Motor()
            msg.motor_id = 0  # 表示使用原始数据模式
            for i in range(8):
                msg.data[i] = self._current_data[i]
            self.publisher_.publish(msg)
            self.get_logger().info(f'已发送原始数据命令: {self._current_data}')
    def periodic_publish(self):
        if self._current_mode == 'current':
            self._publish_current_command()
        elif self._current_mode == 'speed':
            self._publish_speed_command()
        elif self._current_mode == 'raw':
            self._publish_raw_command()

def print_usage():
    print("用法:")
    print("  测试单电机电流控制: python3 motor_test.py current <电机ID 1-4> <电流值 -16384~16384>")
    print("  测试单电机速度控制: python3 motor_test.py speed <电机ID 1-4> <速度值 rpm>")
    print("  测试原始数据控制: python3 motor_test.py raw <8字节十六进制数据，如 AA BB CC DD EE FF 00 11>")

def main():
    if len(sys.argv) < 2:
        print_usage()
        return
    
    rclpy.init()
    test_node = MotorControlTest()
    
    try:
        command_type = sys.argv[1]
        
        if command_type == "current" and len(sys.argv) == 4:
            motor_id = int(sys.argv[2])
            current = int(sys.argv[3])
            
            if not (1 <= motor_id <= 4):
                print("电机ID必须在1-4之间")
                return
            
            if not (-16384 <= current <= 16384):
                print("电流值必须在-16384~16384之间")
                return
            
            test_node.send_current_command(motor_id, current)
            
        elif command_type == "speed" and len(sys.argv) == 4:
            motor_id = int(sys.argv[2])
            speed = int(sys.argv[3])
            
            if not (1 <= motor_id <= 4):
                print("电机ID必须在1-4之间")
                return
            
            test_node.send_speed_command(motor_id, speed)
            
        elif command_type == "raw" and len(sys.argv) >= 3:
            # 解析十六进制数据
            data = []
            for i in range(2, min(len(sys.argv), 10)):  # 最多8个字节
                data.append(int(sys.argv[i], 16))
            
            while len(data) < 8:  # 填充到8字节
                data.append(0)
            
            test_node.send_raw_command(data)
            
        else:
            print_usage()
            return
        
        rclpy.spin(test_node)
            
    except ValueError:
        print("参数格式错误")
    except KeyboardInterrupt:
        print("程序已中断")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()