#include <unistd.h>        // Needed for I2C port
#include <fcntl.h>         // Needed for I2C port
#include <sys/ioctl.h>     // Needed for I2C port
#include <linux/i2c-dev.h> // Needed for I2C port
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class INA219Node : public rclcpp::Node
{
public:
    INA219Node();

    void initialize_i2c();
    double read_bus_voltage(int file_i2c);
    double read_current(int file_i2c);
    double read_power(int file_i2c);
    void publish_values();



private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    int file_i2c;
    double current_lsb;
    double power_lsb;

    int16_t read_i2c_register(int file_i2c, unsigned char reg_address);
};