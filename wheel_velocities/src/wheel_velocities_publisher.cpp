#include "rclcpp/executors.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class VelocityPublisher : public rclcpp::Node {

public:
  std::vector<std_msgs::msg::Float32MultiArray> vec;
  float result[4];
  float (*ptrH)[3] = H;
  float H[4][3];

  VelocityPublisher() : Node("wheel_velocities_publisher") {

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&VelocityPublisher::move_ROSBotXL, this));

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");

    start_H();
  }

  void start_H() {

    float l = 0.165989;
    float w = 0.16256;
    float inverse_r = 1 / 0.05;

    float Homo[4][3] = {{-l - w, 1.0, -1.0},
                        {l + w, 1.0, 1.0},
                        {l + w, 1.0, -1.0},
                        {-l - w, 1.0, 1.0}};

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        H[i][j] = Homo[i][j] * inverse_r;
      }
    }
  }

  void twist2wheels(float vx, float vy, float wz) {

    std::vector<float> vel_axis = {wz, vx, vy};

    std::fill(std::begin(result), std::end(result), 0.0f);

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += ptrH[i][j] * vel_axis[j];
      }
    }
  }

  void move_ROSBotXL() {

    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Move forward");
    twist2wheels(0.25, 0.0, 0.0);
    std_msgs::msg::Float32MultiArray velocities;
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }

    RCLCPP_INFO(this->get_logger(), "Move backward");
    twist2wheels(-0.25, 0.0, 0.0);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }
    RCLCPP_INFO(this->get_logger(), "Move left");

    twist2wheels(0.0, 0.25, 0.0);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }

    RCLCPP_INFO(this->get_logger(), "Move right");
    twist2wheels(0.0, -0.25, 0.0);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }

    RCLCPP_INFO(this->get_logger(), "Move clockwise");
    twist2wheels(0.0, 0.0, -0.25);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }

    RCLCPP_INFO(this->get_logger(), "Move counter-clockwise");
    twist2wheels(0.0, 0.0, 0.25);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }

    RCLCPP_INFO(this->get_logger(), "Stop!");
    twist2wheels(0.0, 0.0, 0.0);
    velocities.data.clear();
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result[i]);
    }
    // Publish
    for (int i = 0; i < 3; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto wheel_velocities_pub_ = std::make_shared<VelocityPublisher>();
  rclcpp::spin(wheel_velocities_pub_);
  rclcpp::shutdown();

  return 0;
}
