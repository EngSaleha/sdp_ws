import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, Motor  # Import Motor class here

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # ROS2 subscription to receive velocity commands
        self.subscription = self.create_subscription(
            Twist, '/diff_cont/cmd_vel_unstamped', self.cmd_to_pwm_callback, 10
        )

        # Define PWM pins for motor speed control (change as needed)
        self.motor_right_pwm = PWMOutputDevice(22)  # GPIO 22 for right motor PWM
        self.motor_left_pwm = PWMOutputDevice(26)   # GPIO 26 for left motor PWM
        
        # Define control pins for motor direction
        self.motor_right = Motor(forward=6, backward=5)  # GPIO pins for right motor direction
        self.motor_left = Motor(forward=19, backward=13)  # GPIO pins for left motor direction

    def cmd_to_pwm_callback(self, msg):
        # Calculate the speed for each wheel
        right_wheel_vel = (msg.linear.x + msg.angular.z) / 2
        left_wheel_vel = (msg.linear.x - msg.angular.z) / 2

        # Convert velocities to PWM (0 to 1 range for PWM control)
        right_pwm = max(0, min(abs(right_wheel_vel), 1))  # Ensure PWM is within [0, 1]
        left_pwm = max(0, min(abs(left_wheel_vel), 1))  # Ensure PWM is within [0, 1]

        # Control the direction and speed of the right motor
        if right_wheel_vel > 0:
            self.motor_right.forward()
        elif right_wheel_vel < 0:
            self.motor_right.backward()
        else:
            self.motor_right.stop()
        
        # Control the direction and speed of the left motor
        if left_wheel_vel > 0:
            self.motor_left.forward()
        elif left_wheel_vel < 0:
            self.motor_left.backward()
        else:
            self.motor_left.stop()

        # Apply PWM control for motor speed (0 to 1 range)
        self.motor_right_pwm.value = right_pwm
        self.motor_left_pwm.value = left_pwm

        self.get_logger().info(f"Left wheel velocity: {left_wheel_vel} | Right wheel velocity: {right_wheel_vel}")
        self.get_logger().info(f"Left PWM: {left_pwm} | Right PWM: {right_pwm}")


def main(args=None):
    rclpy.init(args=args)
    velocity_subscriber = VelocitySubscriber()
    rclpy.spin(velocity_subscriber)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
