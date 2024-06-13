#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stdio.h>

using namespace std::chrono_literals;

class SR04RangeNode : public rclcpp::Node
{
public:
  SR04RangeNode()
      : Node("sr04_range_node"),
        serial1_(io_),
        serial2_(io_),
        deadline_timer1_(io_),
        deadline_timer2_(io_),
        alpha_(0.5),
        filtered_distance1_(0.0),
        filtered_distance2_(0.0),
        current_port_(1)
  {
    this->declare_parameter<std::string>("device1", "/dev/ttySOFTa0");
    this->declare_parameter<std::string>("device2", "/dev/ttySOFTb0");
    this->declare_parameter<std::string>("topic1", "front_left");
    this->declare_parameter<std::string>("topic2", "front_right");
    this->declare_parameter<float>("alpha", 0.5); // LPF alpha parameter

    this->get_parameter("device1", device1_);
    this->get_parameter("device2", device2_);
    this->get_parameter("topic1", topic1_);
    this->get_parameter("topic2", topic2_);
    this->get_parameter("alpha", alpha_);

    RCLCPP_INFO(this->get_logger(), "device1 : %s", device1_.c_str());
    RCLCPP_INFO(this->get_logger(), "device2 : %s", device2_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic1 : %s", topic1_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic2 : %s", topic2_.c_str());
    RCLCPP_INFO(this->get_logger(), "alpha : %f", alpha_);

    // 시리얼 포트 초기화
    open_serial_port(serial1_, device1_);
    open_serial_port(serial2_, device2_);

    range_publisher1_ = this->create_publisher<sensor_msgs::msg::Range>(topic1_, 10);
    range_publisher2_ = this->create_publisher<sensor_msgs::msg::Range>(topic2_, 10);

    ros_timer_ = this->create_wall_timer(
        100ms, std::bind(&SR04RangeNode::toggle_and_publish_range, this));

    start_byte_ = 0xFF;
  }

private:
  void open_serial_port(boost::asio::serial_port &serial, const std::string &device)
  {
    try
    {
      serial.open(device);
      serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", device.c_str(), e.what());
      throw;
    }
  }

  void toggle_and_publish_range()
  {
    if (current_port_ == 1)
    {
      publish_range(serial1_, deadline_timer1_, filtered_distance1_, topic1_, range_publisher1_);
      current_port_ = 2;
    }
    else
    {
      publish_range(serial2_, deadline_timer2_, filtered_distance2_, topic2_, range_publisher2_);
      current_port_ = 1;
    }
  }

  void publish_range(boost::asio::serial_port &serial, boost::asio::deadline_timer &deadline_timer, float &filtered_distance, const std::string &topic, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr &range_publisher)
  {
    std::vector<uint8_t> buffer(4);
    flush_serial_port(serial);

    // 데이터 전송
    int8_t startdata = 0x55;
    boost::asio::write(serial, boost::asio::buffer(&startdata, 1));
    rclcpp::sleep_for(std::chrono::milliseconds(50));

    // Asynchronous read with timeout
    read_completed_ = false;
    boost::asio::async_read(serial, boost::asio::buffer(buffer),
                            [this, &buffer, &filtered_distance, &topic, &range_publisher](const boost::system::error_code &ec, std::size_t length)
                            {
                              if (!ec)
                              {
                                handle_read(ec, length, buffer, filtered_distance, topic, range_publisher);
                                read_completed_ = true;
                                deadline_timer1_.cancel(); // Cancel the timer if read completes
                              }
                            });

    deadline_timer.expires_from_now(boost::posix_time::milliseconds(40)); // Set timeout to 20ms
    deadline_timer.async_wait([this, &serial, &topic](const boost::system::error_code &ec)
                              {
                                if (!ec && !read_completed_)
                                {
                                  // Timer expired before read completed
                                  serial.cancel();
                                  // RCLCPP_WARN(this->get_logger(), "Read operation timed out : %s", topic.c_str());
                                }
                              });

    io_.run();
    io_.reset();
  }

  void handle_read(const boost::system::error_code &ec, std::size_t length, std::vector<uint8_t> &buffer, float &filtered_distance, const std::string &topic, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr &range_publisher)
  {
    if (!ec)
    {
      if (length == 4 && buffer[0] == start_byte_)
      {
        uint16_t high_byte = buffer[1];
        uint16_t low_byte = buffer[2];
        uint8_t checksum = buffer[3];

        uint16_t distance = (high_byte << 8) | low_byte;
        if (checksum == ((start_byte_ + high_byte + low_byte) & 0xFF))
        {
          if (distance == 0)
          {
            RCLCPP_WARN(this->get_logger(), "%s Fail Range Data : %d %d", topic.c_str(), high_byte, low_byte);
            distance = 6000; // Fail-safe distance in mm
          }

          // Apply LPF to the distance
          float current_distance_m = distance / 1000.0; // Convert to meters
          filtered_distance = alpha_ * current_distance_m + (1 - alpha_) * filtered_distance;

          auto message = sensor_msgs::msg::Range();
          message.header.stamp = this->now();
          message.header.frame_id = topic;
          message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          message.field_of_view = 0.1; // 10 degrees
          message.min_range = 0.2;     // 20 cm
          message.max_range = 4.00;    // 4 meters
          message.range = std::min<float>(std::max<float>(filtered_distance, 0.2), 4.0);

          range_publisher->publish(message);
          // RCLCPP_INFO(this->get_logger(), "%s range Data : %f", topic.c_str(), filtered_distance);
        }
      }
    }
    else if (ec != boost::asio::error::operation_aborted)
    {
      RCLCPP_WARN(this->get_logger(), "Error on receive: %s", ec.message().c_str());
    }
  }

  void flush_serial_port(boost::asio::serial_port &serial)
  {
    int fd = serial.native_handle();
    if (tcflush(fd, TCIFLUSH) != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to flush serial port");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher1_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher2_;
  rclcpp::TimerBase::SharedPtr ros_timer_;

  boost::asio::io_service io_;
  boost::asio::serial_port serial1_;
  boost::asio::serial_port serial2_;
  boost::asio::deadline_timer deadline_timer1_;
  boost::asio::deadline_timer deadline_timer2_;
  uint8_t start_byte_;
  bool read_completed_;

  std::string device1_;
  std::string device2_;
  std::string topic1_;
  std::string topic2_;
  float alpha_;
  float filtered_distance1_;
  float filtered_distance2_;
  int current_port_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SR04RangeNode>());
  rclcpp::shutdown();
  return 0;
}
