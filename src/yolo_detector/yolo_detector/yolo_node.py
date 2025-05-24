import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
from ultralytics import YOLO

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_ = self.create_publisher(String, 'yolo_topic', 10)
        self.timer = self.create_timer(0.002, self.timer_callback)

        self.model = YOLO('yolov11_50.pt')
        #url = 'http://192.168.105.11:8080/video'
        self.cap = cv2.VideoCapture(0)

        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.current_frame_index = 0

        self.desired_width = 640
        self.desired_height = 480

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Video sona erdi veya okunamadı.')
            return

        self.current_frame_index += 1

        # Orijinal çözünürlükte çalışıyoruz
        orig_frame = frame.copy()
        h, w, _ = orig_frame.shape

        # Sarı kilitlenme dikdörtgenini çiz (kamera görüntüsü oranlarına göre)
        yellow_left = int(w * 0.25)
        yellow_top = int(h * 0.10)
        yellow_right = int(w * 0.75)
        yellow_bottom = int(h * 0.90)
        cv2.rectangle(orig_frame, (yellow_left, yellow_top), (yellow_right, yellow_bottom), (0, 255, 255), 2)

        # YOLO modelini çalıştırmak için 640x640 olarak pad'li hale getirme
        frame_resized = cv2.resize(frame, (self.desired_width, self.desired_height))
        padded_frame = np.zeros((640, 640, 3), dtype=np.uint8)
        top = (640 - frame_resized.shape[0]) // 2
        left = (640 - frame_resized.shape[1]) // 2
        padded_frame[top:top + frame_resized.shape[0], left:left + frame_resized.shape[1]] = frame_resized

        # YOLO tahmini 640x640'da yapılıyor
        results = self.model.track(padded_frame, conf=0.6, imgsz=640, persist=True)

        detection_sent = False

        for result in results:
            if result.boxes is not None and result.names is not None and len(result.boxes) > 0:
                for box, cls_id in zip(result.boxes.xyxy, result.boxes.cls):
                    x1, y1, x2, y2 = map(int, box)

                    # Tespiti orijinal frame'e geri dönüştür: padding + resize etkisini tersine al
                    x1_unpad = int((x1 - left) * (w / self.desired_width))
                    x2_unpad = int((x2 - left) * (w / self.desired_width))
                    y1_unpad = int((y1 - top) * (h / self.desired_height))
                    y2_unpad = int((y2 - top) * (h / self.desired_height))

                    x_center = (x1_unpad + x2_unpad) // 2
                    y_center = (y1_unpad + y2_unpad) // 2

                    # Orijinal görüntü merkezine göre -100 ile 100 arasında ölçekle
                    x_scaled = ((x_center - w/2) / (w/2)) * 100
                    y_scaled = -((y_center - h/2) / (h/2)) * 100

                    class_id = int(cls_id)
                    label = result.names[class_id]

                    # 🔴 Kırmızı bounding box (ham frame'e çiziliyor)
                    cv2.rectangle(orig_frame, (x1_unpad, y1_unpad), (x2_unpad, y2_unpad), (0, 0, 255), 2)

                    msg_dict = {
                        'label': label,
                        'x_scaled': round(x_scaled, 2),
                        'y_scaled': round(y_scaled, 2),
                        'frame_index': self.current_frame_index,
                        'frame_count': self.frame_count
                    }

                    msg = String()
                    msg.data = json.dumps(msg_dict)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[{self.current_frame_index}/{self.frame_count}] Yayınlandı: {msg.data}")
                    detection_sent = True

        if not detection_sent:
            msg_dict = {
                'label': 'no detections',
                'frame_index': self.current_frame_index,
                'frame_count': self.frame_count
            }
            msg = String()
            msg.data = json.dumps(msg_dict)
            self.publisher_.publish(msg)
            self.get_logger().info(f"[{self.current_frame_index}/{self.frame_count}] Yayınlandı: no detections")

        # 🖼️ Orijinal çözünürlükte ekranda göster
        cv2.imshow("YOLOv8 Detection (HAM FRAME)", orig_frame)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
