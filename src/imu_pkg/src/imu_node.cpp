#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <sstream>

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // ⚠️ IMPORTANT: using ACM0
        serial_fd_ = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }

        configure_serial();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImuNode::read_serial, this));
    }

private:
    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string buffer_;

    void configure_serial()
    {
        struct termios tty;
        tcgetattr(serial_fd_, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(serial_fd_, TCSANOW, &tty);
    }

    void read_serial()
    {
        char temp[256];
        int n = read(serial_fd_, temp, sizeof(temp));

        if (n > 0)
        {
            buffer_.append(temp, n);

            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos)
            {
                std::string line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);

                parse_line(line);
            }
        }
    }

    void parse_line(const std::string &line)
    {
        size_t imu_pos = line.find("IMU");
        if (imu_pos == std::string::npos) return;

        std::string imu_data = line.substr(imu_pos + 3);

        std::stringstream ss(imu_data);

        double gx, gy, gz, ax, ay, az;

        if (!(ss >> gx >> gy >> gz >> ax >> ay >> az))
            return;

        publish_imu(gx, gy, gz, ax, ay, az);
    }

    void publish_imu(double gx, double gy, double gz,
                     double ax, double ay, double az)
    {
        sensor_msgs::msg::Imu msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // Angular velocity
        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        // Linear acceleration
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        // Identity quaternion (NO ORIENTATION)
        msg.orientation.w = 1.0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;

        // Covariance

        // Angular velocity covariance
        msg.angular_velocity_covariance[0] = 0.01;
        msg.angular_velocity_covariance[4] = 0.01;
        msg.angular_velocity_covariance[8] = 0.01;

        // Linear acceleration covariance
        msg.linear_acceleration_covariance[0] = 0.1;
        msg.linear_acceleration_covariance[4] = 0.1;
        msg.linear_acceleration_covariance[8] = 0.1;

        // Orientation covariance (NOT USED)
        msg.orientation_covariance[0] = -1;

        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
