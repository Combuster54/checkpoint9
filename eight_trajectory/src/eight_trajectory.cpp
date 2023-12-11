#include "nav_msgs/msg/detail/odometry__struct.hpp"
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

class EightTrayectoryClass : public rclcpp::Node {

public:
  float hpi;
  std::vector<std_msgs::msg::Float32MultiArray> vec;
  float yaw;
  Eigen::MatrixXd h = Eigen::MatrixXd(4, 3);
  Eigen::Matrix3x3 result = Eigen::MatrixXd(4, 1);

  VelocityPublisher() : Node("wheel_velocities_publisher") {

    h(0, 0) = (-l - w) * inverse_r;
    h(0, 1) = 1.0f * inverse_r;
    h(0, 2) = -1.0f * inverse_r;

    h(1, 0) = (l + w) * inverse_r;
    h(1, 1) = 1.0f * inverse_r;
    h(1, 2) = 1.0f * inverse_r;

    h(2, 0) = (l + w) * inverse_r;
    h(2, 1) = 1.0f * inverse_r;
    h(2, 2) = -1.0f * inverse_r;

    h(3, 0) = (-l - w) * inverse_r;
    h(3, 1) = 1.0f * inverse_r;
    h(3, 2) = 1.0f * inverse_r;

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&EightTrayectoryClass::odom_callback, this, _1),
        options_1);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&EightTrayectoryClass::taking_vectors, this));

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    tf2::Quaternion q;
    float roll, pich;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
  }

  void publish_u_values(const std::vector<float> &vec) {

    twist2wheels(vec(0), vec(1), vec(2));

    std_msgs::msg::Float32MultiArray velocities;
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(result(i, 0));
    }

    // Publish
    for (int i = 0; i < 4; ++i) {
      publisher_->publish(velocities);
      sleep(1);
    }
  }

  void taking_vectors() {
    timer_->cancel();
    for (const auto &doubleVec : allVectors) {
      std::vector<float> floatVec(doubleVec.begin(), doubleVec.end());
      publish_u_values(floatVec);
      // sleep(5);
    }
  }

  void twist2wheels(float wz, float vx, float vy) {

    Eigen::MatrixXd velocities = Eigen::MatrixXd(3, 1);

    velocities(0, 0) = wz;
    velocities(1, 0) = vx;
    velocities(2, 0) = vy;

    result = h * velocities;
  }

private:
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::vector<float> w1 = {0.0, 1, -1};
  std::vector<float> w2 = {0.0, 1, 1};
  std::vector<float> w3 = {0.0, 1, 1};
  std::vector<float> w4 = {1.5708, 1, -1};
  std::vector<float> w5 = {-3.1415, -1, -1};
  std::vector<float> w6 = {0.0, -1, 1};
  std::vector<float> w7 = {0.0, -1, 1};
  std::vector<float> w8 = {0.0, -1, -1};
  std::vector<std::vector<float>> allVectors = {w1, w2, w3, w4, w5, w6, w7, w8};
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto eight_trajectoyry = std::make_shared<EightTrayectoryClass>();
  rclcpp::spin(eight_trajectoyry);
  rclcpp::shutdown();

  return 0;
}
