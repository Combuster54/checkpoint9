#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>
#include <vector>

using Eigen::MatrixXd;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class KinematicModelClass : public rclcpp::Node {

public:
  geometry_msgs::msg::Twist vel_;
  float l = 0.165989;
  float w = 0.16256;
  float inverse_r = 1 / 0.05;

  Eigen::MatrixXd h = Eigen::MatrixXd(4, 3);
  MatrixXd pinv; // 3x4
  Eigen::MatrixXd u = Eigen::MatrixXd(4, 1);
  Eigen::MatrixXd result = Eigen::MatrixXd(3, 1);
  //   float H[4][3] = {{(-l - w) * inverse_r, 1.0f * inverse_r, -1.0f *
  //   inverse_r},
  //                    {(l + w) * inverse_r, 1.0f * inverse_r, 1.0f *
  //                    inverse_r},
  //                    {(l + w) * inverse_r, 1.0f * inverse_r, -1.0f *
  //                    inverse_r},
  //                    {(-l - w) * inverse_r, 1.0f * inverse_r, 1.0f *
  //                    inverse_r}};

  KinematicModelClass() : Node("KinematicModel_Node") {

    // h << (-l - w) * inverse_r, 1.0f, 1.0f * inverse_r,
    //     -1.0f * inverse_r - 1.0f * inverse_r, (l + w) * inverse_r,
    //     1.0f * inverse_r, 1.0f, 1.0f * inverse_r, (l + w) * inverse_r,
    //     1.0f * inverse_r, -1.0f * inverse_r, (-l - w) * inverse_r,
    //     1.0f * inverse_r, 1.0f * inverse_r;

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

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options_1;
    rclcpp::SubscriptionOptions options_2;

    options_1.callback_group = callback_group_;
    options_2.callback_group = callback_group_;

    wheel_speed_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_speed", 10,
            std::bind(&KinematicModelClass::wheel_speed_callback, this, _1),
            options_1);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&KinematicModelClass::start, this));
    psudo_inverse();
  }

  void start() {
    // timer_->cancel();
    result = pinv * u;
    // std::cout << result << std::endl;
    vel_.angular.z = result(0, 0);
    vel_.linear.x = result(1, 0);
    vel_.linear.y = result(2, 0);
    cmd_vel_pub_->publish(vel_);
    RCLCPP_INFO(this->get_logger(), "[FINAL] Wz: %f  Vx: %f  Vy: %f ",
                result(0, 0), result(1, 0), result(2, 0));
  }
  void psudo_inverse() {

    Eigen::CompleteOrthogonalDecomposition<MatrixXd> cqr(h);
    pinv = cqr.pseudoInverse();
    // std::cout << pinv;
  }

  void wheel_speed_callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr float_msg) {

    u(0, 0) = float_msg->data[0];
    u(1, 0) = float_msg->data[1];
    u(2, 0) = float_msg->data[2];
    u(3, 0) = float_msg->data[3];

    // Recieve the Float32Array and see the values for each wheel
    // RCLCPP_INFO(this->get_logger(), "wheel callback");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float phi;
  geometry_msgs::msg::Point position;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;
}; // KinematicModelClass

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto kinematic_model_node = std::make_shared<KinematicModelClass>();
  rclcpp::spin(kinematic_model_node);
  rclcpp::shutdown();

  return 0;
}
