#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_waypoints(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    raw_waypoints = data.get('waypoints')
    if not raw_waypoints:
        raise ValueError(f'no waypoints found in {path}')

    waypoints = []
    for i, wp in enumerate(raw_waypoints):
        if not isinstance(wp, dict):
            raise ValueError(f'waypoint {i} must be a mapping with x/y, got {wp!r}')
        missing = [key for key in ('x', 'y') if key not in wp]
        if missing:
            raise ValueError(f'waypoint {i} is missing required key(s) {missing}: {wp}')
        waypoints.append({
            'x': float(wp['x']),
            'y': float(wp['y']),
            'yaw': float(wp.get('yaw', 0.0)),
        })
    return waypoints


def build_waypoint_markers(waypoints, stamp, frame_id='map'):
    msg = MarkerArray()
    for i, wp in enumerate(waypoints):
        qz, qw = yaw_to_quaternion(wp['yaw'])
        for kind in ('arrow', 'sphere', 'text'):
            marker = Marker()
            marker.header.frame_id = frame_id
            if stamp is not None:
                marker.header.stamp = stamp
            marker.id = len(msg.markers)
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.color.a = 1.0
            if kind == 'arrow':
                marker.type = Marker.ARROW
                marker.scale.x, marker.scale.y, marker.scale.z = 0.3, 0.05, 0.02
                marker.color.g = 1.0
            elif kind == 'sphere':
                marker.type = Marker.SPHERE
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                marker.color.r = 1.0
            else:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.scale.x = marker.scale.y = marker.scale.z = 0.07
                marker.color.g = 1.0
                marker.pose.position.z += 0.2
                marker.text = f'wp_{i + 1}'
            msg.markers.append(marker)
    if not msg.markers:
        clear_all = Marker()
        clear_all.action = Marker.DELETEALL
        msg.markers.append(clear_all)
    return msg


class State(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    PAUSED = 'paused'
    FINISHED = 'finished'


class WaypointNavigator(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='waypoint_navigator')
        # emcl2 は lifecycle ノードではないため waitUntilNav2Active() は呼ばない(デフォルトの amcl/get_state 待ちで無限ループする)
        self.declare_parameter('waypoints_file', '')

        self.waypoints = load_waypoints(self.get_parameter('waypoints_file').value)
        self.state = State.IDLE
        self.start_index = 0
        self.pause_requested = False
        self.pending_start = False
        self.pending_pause = False
        self.pending_resume = False

        # RViz を後から起動しても見えるように latch する。
        # /waypoints は nav2_rviz_plugins の Navigation 2 パネルが使うので、ノード名前空間の下に置く
        self.waypoints_pub = self.create_publisher(
            MarkerArray, '~/waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/pause', self.on_pause)
        self.create_service(Trigger, '~/resume', self.on_resume)

        self.waypoints_pub.publish(
            build_waypoint_markers(self.waypoints, self.get_clock().now().to_msg()))

    # --- サービス: 状態検査 + フラグ設定 + 即時応答のみ。navigator のメソッドはここから呼ばない(ネスト spin 防止) ---

    def on_start(self, request, response):
        if self.state not in (State.IDLE, State.FINISHED):
            response.success = False
            response.message = 'already running'
            return response
        if not self.follow_waypoints_client.server_is_ready():
            response.success = False
            response.message = 'follow_waypoints action server not available'
            return response
        self.pending_start = True
        response.success = True
        return response

    def on_pause(self, request, response):
        if self.state == State.RUNNING:
            self.pending_pause = True
            response.success = True
            return response
        response.success = False
        response.message = 'already paused' if self.state == State.PAUSED else 'not running'
        return response

    def on_resume(self, request, response):
        if self.state == State.PAUSED:
            self.pending_resume = True
            response.success = True
            return response
        response.success = False
        response.message = 'not paused' if self.state == State.RUNNING else 'not running'
        return response

    # --- メインループ ---

    def run(self):
        while rclpy.ok():
            if self.state == State.RUNNING:
                # isTaskComplete() 自体が最大 0.1 秒 spin するのでサービスもここで処理される
                if self.isTaskComplete():
                    self.handle_result()
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.process_flags()

    def process_flags(self):
        if self.pending_start:
            self.pending_start = False
            self.send_from(0)
        if self.pending_pause:
            self.pending_pause = False
            self.pause_requested = True
            self.cancelTask()
        if self.pending_resume:
            self.pending_resume = False
            self.send_from(self.start_index)

    def handle_result(self):
        # pause 要求と走行終端が競合しても取りこぼさないよう、結果処理の冒頭で必ず読み捨てる
        pause_requested = self.pause_requested
        self.pause_requested = False
        result = self.getResult()

        if result == TaskResult.SUCCEEDED:
            missed = self.result_future.result().result.missed_waypoints
            if missed:
                self.get_logger().warn(
                    f'missed waypoints: {[self.start_index + i for i in missed]}')
            self.get_logger().info('finished')
            self.state = State.FINISHED
            return

        if pause_requested:
            self.start_index += self.current_waypoint_index()
            self.get_logger().info(f'paused at waypoint {self.start_index}')
            self.state = State.PAUSED
            return

        self.get_logger().warn('waypoint following failed, stopping')
        self.state = State.IDLE

    def current_waypoint_index(self):
        feedback = self.getFeedback()
        if feedback is None:
            return 0
        # 送信直後の cancel などで範囲外を報告されたら、点を飛ばさないよう先頭から再開する
        remaining = len(self.waypoints) - self.start_index
        return feedback.current_waypoint if 0 <= feedback.current_waypoint < remaining else 0

    def send_from(self, index):
        if not self.follow_waypoints_client.server_is_ready():
            # followWaypoints() はサーバを無限待ちするため、落ちていたら先に検知して IDLE に戻す
            self.get_logger().error('follow_waypoints action server not available, stopping')
            self.state = State.IDLE
            return

        self.start_index = index
        # BasicNavigator はゴール間で feedback をクリアしないため、前回の値が誤適用されるのを防ぐ
        self.feedback = None
        poses = [self.to_pose_stamped(wp) for wp in self.waypoints[index:]]
        self.get_logger().info(f'following {len(poses)} waypoints from index {index}')
        if not self.followWaypoints(poses):
            self.get_logger().error('waypoint goal was rejected')
            self.state = State.IDLE
            return
        self.state = State.RUNNING

    def to_pose_stamped(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        qz, qw = yaw_to_quaternion(wp['yaw'])
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT を受けた rclpy が context を落とすと spin が内部例外で抜ける
        if rclpy.ok():
            raise
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
