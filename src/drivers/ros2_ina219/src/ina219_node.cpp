#include "ina219_driver/ina219_node.h"

using namespace std::chrono_literals;

INA219Node::INA219Node() : Node("ina219_reader"), file_i2c(0)
{
    voltage_publisher_ = this->create_publisher<std_msgs::msg::Float64>("ina219/bus_voltage", 10);
    current_publisher_ = this->create_publisher<std_msgs::msg::Float64>("ina219/current", 10);
    power_publisher_ = this->create_publisher<std_msgs::msg::Float64>("ina219/power", 10);

    int param_calibration = (int)4096;
    double param_rshunt = (double)0.1;

    current_lsb = 0.04096 / (param_calibration * param_rshunt); // from page 12 of INA219 datasheet
    power_lsb = 20 * current_lsb;                               // from page 12 of INA219 datasheet

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&INA219Node::publish_values, this));
    initialize_i2c();
}

void INA219Node::initialize_i2c()
{
    char *filename = (char *)"/dev/i2c-1";
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the i2c bus");
        return;
    }

    int addr = 0x40; // Default I2C address for INA219
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave.");
        return;
    }
}

int16_t INA219Node::read_i2c_register(int file_i2c, unsigned char reg_address)
{
    // Step 1: Write the register address
    unsigned char reg_addr_buffer[1] = {reg_address};
    if (write(file_i2c, reg_addr_buffer, 1) != 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write register address to i2c bus.");
        return -1;
    }

    // Step 2: Read the data from the register
    unsigned char read_buffer[2]; // 2 bytes of data from the register
    if (read(file_i2c, read_buffer, 2) != 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read from the i2c bus.");
        return -1;
    }

    return (read_buffer[0] << 8) + read_buffer[1];
}

double INA219Node::read_bus_voltage(int file_i2c)
{
    // Address of Bus Voltage Register
    const unsigned char BUS_VOLTAGE_REG = 0x02;
    int16_t raw_voltage = read_i2c_register(file_i2c, BUS_VOLTAGE_REG);

    if(raw_voltage == -1) return -1; // error handling
    
    raw_voltage = raw_voltage >> 3; // shift to get actual voltage value
    double actual_voltage = raw_voltage * 0.004; // multiply by 4mV
    RCLCPP_INFO(this->get_logger(), "Bus Voltage: %f V", actual_voltage);

    return actual_voltage;
}

double INA219Node::read_current(int file_i2c)
{
    // Address of Bus Current Register
    const unsigned char BUS_CURRENT_REG = 0x04;
    int16_t raw_current = read_i2c_register(file_i2c, BUS_CURRENT_REG);

    if(raw_current == -1) return -1; // error handling

    double actual_current = raw_current * current_lsb; 
    RCLCPP_INFO(this->get_logger(), "Current: %f A", actual_current);

    return actual_current;
}

double INA219Node::read_power(int file_i2c)
{
    // Address of Bus Power Register
    const unsigned char BUS_POWER_REG = 0x04;
    int16_t raw_power = read_i2c_register(file_i2c, BUS_POWER_REG);

    if(raw_power == -1) return -1; // error handling

    double actual_power = raw_power * power_lsb; 
    RCLCPP_INFO(this->get_logger(), "Power: %f A", actual_power);

    return actual_power;
}

void INA219Node::publish_values()
{
    double voltage = read_bus_voltage(file_i2c);
    std_msgs::msg::Float64 voltage_msg;
    voltage_msg.data = voltage;
    voltage_publisher_->publish(voltage_msg);

    
    double current = read_current(file_i2c);
    std_msgs::msg::Float64 current_msg;
    current_msg.data = current;
    current_publisher_->publish(current_msg);

    double power = read_power(file_i2c);
    std_msgs::msg::Float64 power_msg;
    power_msg.data = power;
    power_publisher_->publish(power_msg);
    
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<INA219Node>());
    rclcpp::shutdown();
    return 0;
}
