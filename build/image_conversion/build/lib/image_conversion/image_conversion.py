import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion')

        self.bridge = CvBridge()

        # Initialize mode variable (1: Greyscale, 2: Color)
        self.mode = 2  # Default mode is Color
        
        # Declare parameters for input and output topics
        self.declare_parameter('input_camera_topic', '/image_raw')
        self.declare_parameter('output_image_topic', '/processed_image')

        # Get parameter values
        input_camera_topic = self.get_parameter('input_camera_topic').get_parameter_value().string_value
        output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

        # Subscribe to the input camera topic
        self.image_sub = self.create_subscription(
            Image,
            input_camera_topic,  # Use parameter for input topic
            self.image_callback,
            10
        )

        # Publisher for the processed image
        self.image_pub = self.create_publisher(
            Image,
            output_image_topic,  # Use parameter for output topic
            10
        )

        # Create a service to change the mode (Greyscale/Color)
        self.srv = self.create_service(
            SetBool,
            'change_image_mode',
            self.change_mode_callback
        )

    def image_callback(self, msg):
        
        try:
          
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.mode == 1:  # Greyscale mode
                processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            else:  # Color mode (no conversion)
                processed_image = cv_image
            
            cv2.imshow("Processed Image", processed_image)
            cv2.waitKey(1)
                        
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8" if self.mode == 2 else "mono8")
         
            self.image_pub.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def change_mode_callback(self, request, response):
       
        if request.data:           
            self.mode = 1
            self.get_logger().info('Mode changed to Greyscale')
        else:     
            self.mode = 2
            self.get_logger().info('Mode changed to Color')

        response.success = True
        response.message = f"Mode set to {('Greyscale' if self.mode == 1 else 'Color')}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

