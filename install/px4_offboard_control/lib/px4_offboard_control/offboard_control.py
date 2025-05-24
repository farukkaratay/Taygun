#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, ActuatorControls, VehicleCommand, VehicleStatus
import time


class PWMControl(Node):
    """PX4 Offboard kontrol düğümü — throttle aşamalı artış"""

    def __init__(self):
        super().__init__('pwm_control_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.actuator_controls_publisher = self.create_publisher(
            ActuatorControls, '/fmu/in/actuator_controls_0', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_status = VehicleStatus()

        self.armed = False
        self.offboard_started = False
        self.start_time = None

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def send_vehicle_command(self, command, **params):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = params.get("param1", 0.0)
        cmd.param2 = params.get("param2", 0.0)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arming gönderildi.')

    def start_offboard(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard başlatıldı.')

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_mode()

        # Offboard ve arm henüz başlatılmadıysa başlat
        if not self.offboard_started:
            self.start_offboard()
            time.sleep(0.5)
            self.arm()
            time.sleep(0.5)
            self.start_time = time.time()
            self.offboard_started = True
            self.get_logger().info("Offboard ve Arm tamam. Throttle artışı başlıyor...")

        # Zaman kontrolü ve throttle artışı
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > 10:
                self.throttle = 0.6
            elif elapsed > 5:
                self.throttle = 0.5
            else:
                self.throttle = 0.4

            self.send_pwm_control(self.roll, self.pitch, self.yaw, self.throttle)

    def send_pwm_control(self, roll, pitch, yaw, throttle):
        msg = ActuatorControls()
        msg.control = [roll, pitch, yaw, throttle, 0.0, 0.0, 0.0, 0.0]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_controls_publisher.publish(msg)
        self.get_logger().info(f"PWM: Roll={roll}, Pitch={pitch}, Yaw={yaw}, Throttle={throttle:.2f}")


def main(args=None):
    rclpy.init(args=args)
    pwm_control_node = PWMControl()
    rclpy.spin(pwm_control_node)
    pwm_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
