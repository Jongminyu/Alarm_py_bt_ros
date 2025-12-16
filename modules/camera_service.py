import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import cv2
import os
from datetime import datetime
import sys

# Safe Import for cv_bridge
try:
    from cv_bridge import CvBridge
except ImportError:
    print("❌ ERROR: cv_bridge not found! Install it via: sudo apt install ros-<distro>-cv-bridge")
    sys.exit(1)

class CameraServiceNode(Node):
    def __init__(self):
        super().__init__('camera_service_node')
        
        # Subscribe to camera topic 
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.topic = self.get_parameter('camera_topic').value
        
        self.subscription = self.create_subscription(
            Image,
            self.topic, 
            self.listener_callback,
            10)
        self.latest_image = None
        self.bridge = CvBridge()

        # Create Service
        self.srv = self.create_service(Trigger, 'capture_image_service', self.capture_callback)
        self.get_logger().info(f'📸 Camera Service Ready! Listening on {self.topic}')

    def listener_callback(self, msg):
        # self.get_logger().info('Received image frame') # Debug
        self.latest_image = msg

    def capture_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = "No image received yet! Check camera connection."
            self.get_logger().warn(response.message)
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Generate Filename (YYYYMMDD_HHMMSS)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"mission_success_{timestamp}.jpg"
            
            # Save to 'captures' folder
            save_dir = os.path.join(os.getcwd(), 'captures')
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, cv_image)
            
            response.success = True
            response.message = f"Saved image to {filepath}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        if rclpy.ok():
             node.destroy_node()
             rclpy.shutdown()

if __name__ == '__main__':
    main()
