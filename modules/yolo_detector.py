import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('conf_threshold', 0.5)
        
        model_name = self.get_parameter('model').value
        camera_topic = self.get_parameter('camera_topic').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        
        # Initialize YOLO
        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)
        self.get_logger().info('YOLO model loaded.')
        
        # ROS Communication
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        
        self.debug_pub = self.create_publisher(Image, '/yolo/debug_image', 10)
        self.person_detected_pub = self.create_publisher(Bool, '/yolo/person_detected', 10)
        self.detection_info_pub = self.create_publisher(String, '/yolo/detection_info', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info(f'YoloDetector started, listening on {camera_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Inference
        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
        
        person_detected = False
        detections = []
        
        # Process results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                if self.model.names[cls] == 'person':
                    person_detected = True
                    
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    
                    # Calculate center
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    
                    detections.append({
                        'x': float(cx),
                        'y': float(cy),
                        'w': float(x2 - x1),
                        'h': float(y2 - y1),
                        'conf': conf
                    })
                    
                    # Draw on image
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'Person {conf:.2f}', (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish results
        self.person_detected_pub.publish(Bool(data=person_detected))
        
        # Always publish detection info (empty list if none)
        info_msg = json.dumps(detections)
        self.detection_info_pub.publish(String(data=info_msg))
            
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Publish debug image error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
