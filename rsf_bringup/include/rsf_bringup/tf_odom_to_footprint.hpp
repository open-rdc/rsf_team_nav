#ifndef RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_
#define RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rsf_bringup
{

// odom はセンサ基準で配信されるため、そのまま合成すると odom 原点がセンサ取り付け
// 高さに置かれ、base_footprint が地面から沈む。高さを落として odom を地面に合わせる。
inline tf2::Transform compute_odom_to_base(
  const tf2::Transform & odom_to_sensor,
  const tf2::Transform & base_to_sensor)
{
  tf2::Transform odom_to_base = odom_to_sensor * base_to_sensor.inverse();
  tf2::Vector3 origin = odom_to_base.getOrigin();
  origin.setZ(0.0);
  odom_to_base.setOrigin(origin);
  return odom_to_base;
}

class TfOdomToFootprint : public rclcpp::Node
{
public:
  TfOdomToFootprint();

private:
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::string odom_frame_;
  std::string sensor_frame_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}

#endif
