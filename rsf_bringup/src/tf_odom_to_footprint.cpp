#include "rsf_bringup/tf_odom_to_footprint.hpp"

#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rsf_bringup
{

TfOdomToFootprint::TfOdomToFootprint()
: Node("tf_odom_to_footprint"),
  odom_frame_(declare_parameter<std::string>("odom_frame", "rsf_odom")),
  sensor_frame_(declare_parameter<std::string>("sensor_frame", "rsf_hokuyo3d")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_footprint")),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(*this)
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/rsf/rsf_odom");
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    input_topic,
    rclcpp::QoS(10),
    std::bind(&TfOdomToFootprint::odometry_callback, this, std::placeholders::_1));
}

void TfOdomToFootprint::odometry_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (msg->header.frame_id != odom_frame_) {
    RCLCPP_ERROR(
      get_logger(), "Expected frame_id '%s', got '%s'",
      odom_frame_.c_str(), msg->header.frame_id.c_str());
    return;
  }
  if (msg->child_frame_id != sensor_frame_) {
    RCLCPP_ERROR(
      get_logger(), "Expected child_frame_id '%s', got '%s'",
      sensor_frame_.c_str(), msg->child_frame_id.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped base_to_sensor_msg;
  try {
    base_to_sensor_msg = tf_buffer_.lookupTransform(
      base_frame_, sensor_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(
      get_logger(), "Cannot transform '%s' to '%s': %s",
      sensor_frame_.c_str(), base_frame_.c_str(), error.what());
    return;
  }

  tf2::Transform odom_to_sensor;
  tf2::fromMsg(msg->pose.pose, odom_to_sensor);

  tf2::Transform base_to_sensor;
  tf2::fromMsg(base_to_sensor_msg.transform, base_to_sensor);

  const tf2::Transform odom_to_base = compute_odom_to_base(odom_to_sensor, base_to_sensor);

  geometry_msgs::msg::TransformStamped output;
  output.header.stamp = msg->header.stamp;
  output.header.frame_id = odom_frame_;
  output.child_frame_id = base_frame_;
  output.transform = tf2::toMsg(odom_to_base);
  tf_broadcaster_.sendTransform(output);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rsf_bringup::TfOdomToFootprint>());
  rclcpp::shutdown();
  return 0;
}
