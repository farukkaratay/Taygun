import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class YoloListener(Node):
    def __init__(self):
        super().__init__('yolo_listener')
        self.subscription = self.create_subscription(
            String,
            'yolo_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            label = data.get('label', 'unknown')
            frame_index = data.get('frame_index', 0)
            frame_count = data.get('frame_count', 0)

            if label == 'no detections':
                self.get_logger().info(f"[{frame_index}/{frame_count}] Tespit yok.")
            else:
                x_scaled = data.get('x_scaled', 0.0)
                y_scaled = data.get('y_scaled', 0.0)

                self.get_logger().info(
                    f"[{frame_index}/{frame_count}] Nesne: {label}, "
                    f"Ölçeklenmiş merkez: ({x_scaled}, {y_scaled})"
                )
        except json.JSONDecodeError:
            self.get_logger().error('Mesaj JSON formatında değil.')

def main(args=None):
    rclpy.init(args=args)
    node = YoloListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
