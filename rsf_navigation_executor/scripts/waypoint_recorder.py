#!/usr/bin/env python3
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('output_file', '')
        self.declare_parameter('record_interval', 5.0)
        self.declare_parameter('min_distance', 0.5)

        self.latest_pose = None
        self.waypoints = []

        self.create_subscription(
            PoseWithCovarianceStamped, 'mcl_pose', self.on_pose, 10)
        self.create_timer(
            self.get_parameter('record_interval').value, self.on_record_timer)
        self.create_service(Trigger, '~/save', self.on_save)

        self.get_logger().info(
            'save with: ros2 service call '
            f'{self.resolve_service_name("~/save")} std_srvs/srv/Trigger')

    def on_pose(self, msg):
        self.latest_pose = msg.pose.pose

    def on_record_timer(self):
        if self.latest_pose is None:
            self.get_logger().warn('no pose received yet')
            return

        x = self.latest_pose.position.x
        y = self.latest_pose.position.y
        if self.waypoints:
            last = self.waypoints[-1]
            if math.hypot(x - last['x'], y - last['y']) < self.get_parameter('min_distance').value:
                return

        self.waypoints.append({
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': round(quaternion_to_yaw(self.latest_pose.orientation), 3),
        })
        self.get_logger().info(f'recorded waypoint {len(self.waypoints) - 1}')

    def on_save(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'no waypoints recorded'
            return response

        output_file = os.path.abspath(self.get_parameter('output_file').value)
        try:
            with open(output_file, 'w') as f:
                yaml.safe_dump({'waypoints': self.waypoints}, f, sort_keys=False)
        except OSError as error:
            response.success = False
            response.message = f'failed to write {output_file}: {error}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f'saved {len(self.waypoints)} waypoints to {output_file}'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT を受けた rclpy が context を落とすと spin が内部例外で抜ける
        if rclpy.ok():
            raise
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
