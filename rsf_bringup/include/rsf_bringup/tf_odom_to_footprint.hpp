#ifndef RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_
#define RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

inline tf2::Quaternion level_orientation(const tf2::Quaternion & lio, tf2::Vector3 up)
{
  up.normalize();
  const tf2::Vector3 z(0.0, 0.0, 1.0);
  const tf2::Vector3 axis = up.cross(z);
  const double s = axis.length();
  tf2::Quaternion align = tf2::Quaternion::getIdentity();
  if (s > 1e-9) {
    align = tf2::Quaternion(axis / s, std::atan2(s, up.dot(z)));
  }
  const tf2::Vector3 forward = tf2::quatRotate(align, tf2::Vector3(1.0, 0.0, 0.0));
  const tf2::Vector3 lio_forward = tf2::quatRotate(lio, tf2::Vector3(1.0, 0.0, 0.0));
  tf2::Quaternion yaw;
  yaw.setRPY(
    0.0, 0.0,
    std::atan2(lio_forward.y(), lio_forward.x()) - std::atan2(forward.y(), forward.x()));
  return yaw * align;
}

class TfOdomToFootprint : public rclcpp::Node
{
public:
  TfOdomToFootprint();

private:
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  std::string odom_frame_;
  std::string sensor_frame_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  tf2::Matrix3x3 imu_to_sensor_;
  double accel_time_constant_;
  tf2::Vector3 gravity_;
  rclcpp::Time gravity_stamp_;
  bool has_gravity_ = false;
};

}

#endif
