#include "rsf_bringup/tf_odom_to_footprint.hpp"

#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
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
  const auto m = declare_parameter<std::vector<double>>(
    "imu_to_sensor", {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0});
  imu_to_sensor_.setValue(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  accel_time_constant_ = declare_parameter<double>("accel_time_constant", 1.0);
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "/rsf/imu");
  if (!imu_topic.empty()) {
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&TfOdomToFootprint::imu_callback, this, std::placeholders::_1));
  }
}

void TfOdomToFootprint::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto & a = msg->linear_acceleration;
  const tf2::Vector3 accel = imu_to_sensor_ * tf2::Vector3(a.x, a.y, a.z);
  const rclcpp::Time stamp(msg->header.stamp);
  if (!has_gravity_) {
    gravity_ = accel;
    has_gravity_ = true;
  } else {
    const double dt = std::max(0.0, (stamp - gravity_stamp_).seconds());
    const double alpha = std::clamp(dt / accel_time_constant_, 0.0, 1.0);
    gravity_ += alpha * (accel - gravity_);
  }
  gravity_stamp_ = stamp;
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
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for transform from '%s' to '%s': %s",
      sensor_frame_.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  tf2::Transform odom_to_sensor;
  tf2::fromMsg(msg->pose.pose, odom_to_sensor);
  if (has_gravity_ && gravity_.length() > 1e-3) {
    odom_to_sensor.setRotation(level_orientation(odom_to_sensor.getRotation(), gravity_));
  }

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
