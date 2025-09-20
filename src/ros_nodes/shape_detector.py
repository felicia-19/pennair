import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .detection_msg import DetectionMsg
from ..shape_detection.utils import detect_shapes, draw_shapes

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        self.subscription = self.create_subscription(
            Image, 'video_frames', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(DetectionMsg, 'shape_detections', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.bitwise_not(mask)
        shapes = detect_shapes(mask, min_area=500)

        for (cx, cy), _ in shapes:
            detection = DetectionMsg()
            detection.x = float(cx)
            detection.y = float(cy)
            self.publisher_.publish(detection)

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()