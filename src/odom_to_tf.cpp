#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTf : public rclcpp::Node {
 public:
  OdomToTf() : Node("francor_odom_to_tf_node") {
    this->declare_parameter("odom_stamp_offset", 0.0);
    _odom_stamp_offset = this->get_parameter("odom_stamp_offset").as_double();

    _clock = this->get_clock();
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(1).best_effort(), std::bind(&OdomToTf::cbOdomMsg, this, std::placeholders::_1));
  }
  virtual ~OdomToTf() = default;

 private:
  void cbOdomMsg(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;

    t.header = msg->header;
    t.header.stamp = this->get_clock()->now() + rclcpp::Duration::from_seconds(_odom_stamp_offset);

    t.child_frame_id = msg->child_frame_id;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    _tf_broadcaster->sendTransform(t);
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  rclcpp::Logger _logger{rclcpp::get_logger("francor_odom_to_tf_node")};

  rclcpp::Clock::SharedPtr _clock;
  double _odom_stamp_offset;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTf>());
  rclcpp::shutdown();
  return 0;
}