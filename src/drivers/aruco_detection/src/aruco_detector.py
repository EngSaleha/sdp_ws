import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros2_aruco_interfaces.msg import ArucoMarkers

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Subscribe to /aruco_markers
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10)
        
        # Publisher for /aruco_detected
        self.publisher = self.create_publisher(Bool, '/aruco_detected', 10)

    def aruco_callback(self, msg):
        detected = bool(msg.marker_ids)  # True if markers are detected
        detection_msg = Bool()
        detection_msg.data = detected
        self.publisher.publish(detection_msg)
        self.get_logger().info(f'Aruco detected: {detected}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
