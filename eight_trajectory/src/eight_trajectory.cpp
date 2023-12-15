#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;
using Eigen::MatrixXd;

using std::placeholders::_1;

class EightTrayectoryClass : public rclcpp::Node {

public:
  float hpi;
  std::vector<std_msgs::msg::Float32MultiArray> vec;
  double yaw = 0.0;
  Eigen::MatrixXd h = Eigen::MatrixXd(4, 3);
  Eigen::MatrixXd u = Eigen::MatrixXd(4, 1);

  Eigen::MatrixXd r = Eigen::MatrixXd(3, 3);
  float l = 0.165989;
  float w = 0.16256;
  float inverse_r = 1 / 0.05;

  float wz = 0.0, vx = 0.0, vy = 0.0;

  EightTrayectoryClass() : Node("EightTrajectory_Node") {

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
        "odometry/filtered", 10,
        std::bind(&EightTrayectoryClass::odom_callback, this, _1));

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&EightTrayectoryClass::taking_vectors, this));

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");
  }

  void velocity2twist(float dphi, float dx, float dy) {

    r(0, 0) = 1.0;
    r(0, 1) = 0.0;
    r(0, 2) = 0.0;

    r(1, 0) = 0.0;
    r(1, 1) = std::cos(yaw);
    r(1, 2) = std::sin(yaw);

    r(2, 0) = 0.0;
    r(2, 1) = -1 * std::sin(yaw);
    r(2, 2) = std::cos(yaw);

    Eigen::MatrixXd v = Eigen::MatrixXd(3, 1);

    v(0, 0) = dphi;
    v(1, 0) = dx;
    v(2, 0) = dy;

    Eigen::MatrixXd twist = Eigen::MatrixXd(3, 1);

    twist = r * v;
    wz = twist(0, 0);
    vx = twist(1, 0);
    vy = twist(2, 0);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {

    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

    double roll, pitch;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "[Yaw]: %f ", yaw);
  }

  void publish_u_values(const std::vector<float> &vec) {

    u.setZero();
    velocity2twist(vec[0], vec[1], vec[2]);
    RCLCPP_INFO(this->get_logger(), "[velocity2twist] Wz: %f  Vx: %f  Vy: %f ",
                wz, vx, vy);
    twist2wheels(wz, vx, vy);
    RCLCPP_INFO(this->get_logger(),
                "[twist2wheels] u1: %f  u2: %f  u3: %f u4: %f ", u(0, 0),
                u(1, 0), u(2, 0), u(3, 0));
    std_msgs::msg::Float32MultiArray velocities;
    for (int i = 0; i < 4; ++i) {
      velocities.data.push_back(u(i, 0));
    }

    // Publish
    for (int i = 0; i < 2500; ++i) {

      if (i > 2300) {
        std_msgs::msg::Float32MultiArray vel_zero;
        vel_zero.data.push_back(0.0);
        vel_zero.data.push_back(0.0);
        vel_zero.data.push_back(0.0);
        vel_zero.data.push_back(0.0);
        publisher_->publish(vel_zero);

      } else {

        publisher_->publish(velocities);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void taking_vectors() {
    timer_->cancel();
    for (const auto &doubleVec : allVectors) {
      std::vector<float> floatVec(doubleVec.begin(), doubleVec.end());
      publish_u_values(floatVec);
    }
  }

  void twist2wheels(float wz, float vx, float vy) {

    Eigen::MatrixXd velocities = Eigen::MatrixXd(3, 1);

    velocities(0, 0) = wz;
    velocities(1, 0) = vx;
    velocities(2, 0) = vy;

    u = h * velocities;
  }

private:
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  std::vector<float> w0 = {0.0, 0.0, 0.0};
  std::vector<float> w1 = {0.0, 1.0, -1.0};
  std::vector<float> w2 = {0.0, 1.0, 1.0};
  std::vector<float> w3 = {0.0, 1.0, 1.0};
  std::vector<float> w4 = {1.5708, 1.0, -1.0};
  std::vector<float> w5 = {-3.1415, -1.0, -1.0};
  std::vector<float> w6 = {0.0, -1.0, 1.0};
  std::vector<float> w7 = {0.0, -1.0, 1.0};
  std::vector<float> w8 = {0.0, -1.0, -1.0};
  std::vector<float> w9 = {0.0, 0.0, 0.0};

  std::vector<std::vector<float>> allVectors = {w0, w1, w2, w3, w4,
                                                w5, w6, w7, w8, w9};
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto eight_trajectoyry = std::make_shared<EightTrayectoryClass>();
  rclcpp::spin(eight_trajectoyry);
  rclcpp::shutdown();

  return 0;
}