#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from px4_msgs.msg import VehicleAttitudeSetpoint
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
from std_msgs.msg import String  # String mesajını import ettik

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        self.roll_deg = 0.0   # Roll (derece cinsinden)
        self.pitch_deg = 30.0   # Pitch (derece cinsinden)
        self.yaw_deg = 0.0     # Yaw (derece cinsinden)
        self.throttle = 0.9    # Motor gücü [0-1] arası
        
        # Gelen verileri dinleyecek subscriber
        self.subscription = self.create_subscription(
            String,
            'yolo_topic',  # YoloPublisher'dan gelen topic
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', 10)
        self.timer = self.create_timer(0.005, self.publish_attitude_setpoint)

    def listener_callback(self, msg):
        """Dinleyici, YoloPublisher'dan gelen verileri alır ve günceller."""
        try:
            data = json.loads(msg.data)
            label = data.get('label', 'unknown')
            if label != 'no detections':  # Eğer nesne tespit edildiyse
                self.yaw_deg = data.get('x_scaled', 0.0)  # X pozisyonu, yaw olarak kullanılabilir
                self.pitch_deg = -1 * data.get('y_scaled', 30.0)  # Y pozisyonu, pitch olarak kullanılabilir
                self.get_logger().info(f"Gelen değerler -> Yaw: {self.yaw_deg}, Pitch: {self.pitch_deg}")
        except json.JSONDecodeError:
            self.get_logger().error('Mesaj JSON formatında değil.')

    def publish_attitude_setpoint(self):
        """Uçuş yönlerini gönderir."""
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # Mikro saniye cinsinden zaman damgası

        # Roll, Pitch, Yaw değerlerini radyana çevir
        roll = np.radians(self.roll_deg)
        pitch = np.radians(self.pitch_deg)
        yaw = np.radians(self.yaw_deg)

        # Euler açılarını Quaternion'a çevir
        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # [x, y, z, w] formatında

        msg.q_d = [q[0], q[1], q[2], q[3]]  # Quaternion setpoint
        msg.thrust_body = [self.throttle, 0.0, 0.0]  # Thrust (Throttle)
        msg.reset_integral = False  # İntgeralleri sıfırlamadan devam etsin
        msg.fw_control_yaw_wheel = False  # Sabit kanatlı uçak için kullanılmaz

        self.publisher.publish(msg)
        self.get_logger().info(f"Setpoint Gönderildi -> Roll: {self.roll_deg}°, Pitch: {self.pitch_deg}°, Yaw: {self.yaw_deg}°, Throttle: {self.throttle}")

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    print('Offboard control node stopped.')

    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)