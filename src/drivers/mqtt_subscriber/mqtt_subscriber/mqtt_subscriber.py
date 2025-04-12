import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Float32

class MqttSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_subscriber')

        # Create ROS2 publishers
        self.distance_publisher = self.create_publisher(Float32, 'ultrasonic/distance', 10)
        self.status_publisher = self.create_publisher(String, 'ultrasonic/status', 10)

        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to MQTT broker
        mqtt_broker = "192.168.86.39"  # Replace with your MQTT broker IP
        self.mqtt_client.connect(mqtt_broker, 1883, 60)

        # Start MQTT loop
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        # Subscribe to both topics
        client.subscribe("sensor/ultrasonic/distance")
        client.subscribe("sensor/ultrasonic/status")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == "sensor/ultrasonic/distance":
            try:
                distance_msg = Float32()
                distance_msg.data = float(payload)
                self.distance_publisher.publish(distance_msg)
                self.get_logger().info(f"Published Distance: {payload} cm")
            except ValueError:
                self.get_logger().error(f"Invalid distance value received: {payload}")

        elif topic == "sensor/ultrasonic/status":
            status_msg = String()
            status_msg.data = payload
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f"Published Status: {payload}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

