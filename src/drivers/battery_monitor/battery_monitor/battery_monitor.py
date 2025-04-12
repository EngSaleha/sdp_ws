import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Subscribe to the correct topic
        self.subscription = self.create_subscription(
            Float64,
            '/ina219/bus_voltage',  # Updated topic name
            self.battery_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.low_battery_threshold = 15.94  # Adjust threshold as needed

    def battery_callback(self, msg):
        voltage = msg.data
        self.get_logger().info(f'Battery Voltage: {voltage:.2f}V')

        if voltage < self.low_battery_threshold:
            self.get_logger().warn(f'Battery low: {voltage:.2f}V! Charging needed.')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

