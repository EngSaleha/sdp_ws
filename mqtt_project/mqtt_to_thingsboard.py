import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import paho.mqtt.client as mqtt
import json

# ThingsBoard MQTT details
THINGSBOARD_HOST = "demo.thingsboard.io"  # Change if using a local instance
ACCESS_TOKEN = "X1wgoyP6kkjMfd9YVxPO"  # Replace with your ThingsBoard device token
MQTT_TOPIC = "v1/devices/me/telemetry"  # Default telemetry topic

# Battery voltage threshold (Adjust as needed)
LOW_VOLTAGE_THRESHOLD = 15.6  # Example: If voltage < 12.0V, charging is needed

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor_node")
        
        # Subscribe to the ROS2 topic for battery voltage
        self.subscription = self.create_subscription(
            Float64,
            "/ina219/bus_voltage",  # Ensure this matches your actual ROS2 topic
            self.battery_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Setup MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(ACCESS_TOKEN)
        self.mqtt_client.on_connect = self.on_connect

        try:
            self.mqtt_client.connect(THINGSBOARD_HOST, 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("Connected to ThingsBoard")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ThingsBoard: {e}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT Connected successfully")
        else:
            self.get_logger().error(f"MQTT Connection failed with code {rc}")

    def battery_callback(self, msg):
        battery_voltage = msg.data
        charging_needed = battery_voltage < LOW_VOLTAGE_THRESHOLD

        if charging_needed:
            status_message = "Battery low! Charging needed."
        else:
            status_message = "Battery level sufficient."

        # Log the battery status
        self.get_logger().info(f"Battery Voltage: {battery_voltage}V | {status_message}")

        # Prepare the data payload
        data = {
            "battery_voltage": battery_voltage,
            "charging_needed": charging_needed
        }

        # Send data to ThingsBoard
        try:
            self.mqtt_client.publish(MQTT_TOPIC, json.dumps(data))
            self.get_logger().info(f"Data sent to ThingsBoard: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")

def main():
    rclpy.init()
    node = BatteryMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


