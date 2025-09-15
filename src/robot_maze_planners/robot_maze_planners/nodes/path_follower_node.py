import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Parameters
        params = {
            'linear_gain': 0.8,
            'angular_gain': 2.0,
            'goal_tolerance': 0.15,
            'max_linear_speed': 0.6,
            'max_angular_speed': 1.2,
            'odom_topic': '/model/slambot/odom',
            'lookahead_distance': 0.6,
            'min_linear_speed': 0.08,
            'ang_slowdown_threshold': 1.2,
            # Stuck / recovery tuning
            'no_progress_timeout': 2.5,
            'recovery_duration': 1.2,
            'recovery_forward_speed': 0.15,
            'progress_min_delta': 0.03,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)

        # Parameter values
        gp = self.get_parameter
        self.linear_gain = gp('linear_gain').get_parameter_value().double_value
        self.angular_gain = gp('angular_gain').get_parameter_value().double_value
        self.goal_tolerance = gp('goal_tolerance').get_parameter_value().double_value
        self.max_linear_speed = gp('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = gp('max_angular_speed').get_parameter_value().double_value
        self.odom_topic = gp('odom_topic').get_parameter_value().string_value or '/model/slambot/odom'
        self.lookahead_distance = gp('lookahead_distance').get_parameter_value().double_value or 0.5
        self.min_linear_speed = gp('min_linear_speed').get_parameter_value().double_value or 0.05
        self.ang_slowdown_threshold = gp('ang_slowdown_threshold').get_parameter_value().double_value or 0.8
        self.no_progress_timeout = gp('no_progress_timeout').get_parameter_value().double_value or 2.5
        self.recovery_duration = gp('recovery_duration').get_parameter_value().double_value or 1.2
        self.recovery_forward_speed = gp('recovery_forward_speed').get_parameter_value().double_value or 0.15
        self.progress_min_delta = gp('progress_min_delta').get_parameter_value().double_value or 0.03

        # State
        self.path = []
        self.current_idx = 0
        self.pose = None
        self._stopped = False
        self._last_progress_time = self.get_clock().now().nanoseconds / 1e9
        self._last_progress_dist = float('inf')
        self._in_recovery_until = 0.0
        self._recovery_count = 0
        self._last_cmd_linear = 0.0

        # Subscriptions
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        if self.odom_topic != '/odom':
            self.odom_sub_alt = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Publishers (model-specific and generic)
        self.cmd_pub_model = self.create_publisher(Twist, '/model/slambot/cmd_vel', 10)
        self.cmd_pub_generic = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop timer
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('Path follower started')

    def path_cb(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if new_path == self.path and self.path:
            return  # ignore identical re-publications
        # Swap path and preserve progress by snapping index to nearest point on the new path
        self.path = new_path
        if self.pose is not None and self.path:
            px, py, _ = self.pose
            idx = self._nearest_index(px, py)
            self.current_idx = idx if idx is not None else 0
        else:
            self.current_idx = 0
        self._stopped = False
        self.get_logger().info(f'Received planned path with {len(self.path)} points')

    def odom_cb(self, msg: Odometry):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quat(msg.pose.pose.orientation),
        )
        if not hasattr(self, '_logged_first_odom'):
            self.get_logger().info(
                f"First odom pose=({self.pose[0]:.3f},{self.pose[1]:.3f},{self.pose[2]:.3f})"
            )
            self._logged_first_odom = True
        elif not hasattr(self, '_odom_counter'):
            self._odom_counter = 0
        else:
            self._odom_counter += 1
            if self._odom_counter % 10 == 0:
                self.get_logger().debug(
                    f"Odom pose=({self.pose[0]:.3f},{self.pose[1]:.3f},{self.pose[2]:.3f}) path_len={len(self.path)} idx={self.current_idx}"
                )

    def _yaw_from_quat(self, q):
        # quaternion to yaw
        x, y, z, w = q.x, q.y, q.z, q.w
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def timer_cb(self):
        if not self.path:
            if self.pose is not None and not self._stopped:
                self.get_logger().debug('No path available yet; holding position.')
            return
        if self.pose is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # clamp current index
        if self.current_idx >= len(self.path):
            # reached end
            if not self._stopped:
                stop_msg = Twist()
                self.cmd_pub_model.publish(stop_msg)
                self.cmd_pub_generic.publish(stop_msg)
                self.get_logger().info('Reached final waypoint; stopping.')
                self._stopped = True
            return

        px, py, yaw = self.pose
        # 1) Snap to nearest path index to avoid trying to go backwards
        nearest_idx = self._nearest_index(px, py)
        if nearest_idx is not None and nearest_idx > self.current_idx:
            self.current_idx = min(nearest_idx, len(self.path) - 1)

        # 2) Choose a lookahead target beyond current_idx
        target_idx = self.current_idx
        accum = 0.0
        lastx, lasty = self.path[target_idx]
        for i in range(self.current_idx + 1, len(self.path)):
            x, y = self.path[i]
            accum += math.hypot(x - lastx, y - lasty)
            lastx, lasty = x, y
            if accum >= self.lookahead_distance:
                target_idx = i
                break
        tx, ty = self.path[target_idx]

        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        ang_error = self._angle_diff(angle_to_target, yaw)

        # If close to final goal overall, stop
        if target_idx >= len(self.path) - 1 and dist < self.goal_tolerance:
            self.current_idx = len(self.path)
            stop_msg = Twist()
            self.cmd_pub_model.publish(stop_msg)
            self.cmd_pub_generic.publish(stop_msg)
            self.get_logger().info('Goal reached; stopping.')
            self._stopped = True
            return

        # Advance current_idx if we pass intermediate waypoints
        while self.current_idx < target_idx:
            wx, wy = self.path[self.current_idx]
            if math.hypot(wx - px, wy - py) < self.goal_tolerance:
                self.current_idx += 1
            else:
                break

        # 3) Control law with angular-aware linear scaling
        twist = Twist()

        # Recovery behavior if stuck (no progress)
        if (self._last_progress_dist - dist) > self.progress_min_delta:
            self._last_progress_dist = dist
            self._last_progress_time = now_sec
        elif (now_sec - self._last_progress_time > self.no_progress_timeout
              and dist > max(self.goal_tolerance * 2.0, 0.3)):
            # trigger a brief arc-turn recovery
            self._in_recovery_until = now_sec + self.recovery_duration
            self._last_progress_time = now_sec
            self._recovery_count += 1
            # Skip a potentially-bad waypoint to avoid deadlock at tight corners
            if self.current_idx < len(self.path) - 1:
                self.current_idx += 1
            self.get_logger().info(
                f'No progress detected; entering arc-turn recovery for {self.recovery_duration:.1f}s'
            )

        if now_sec < self._in_recovery_until:
            # Arc-turn recovery: turn while slowly moving to avoid wall lock
            # Alternate forward/backward to escape if nose is pressed to a wall
            direction = 1.0 if (self._recovery_count % 2 == 1) else -1.0
            twist.linear.x = direction * self.recovery_forward_speed
            twist.angular.z = self.max_angular_speed * (1.0 if ang_error >= 0.0 else -1.0)
            self.cmd_pub_model.publish(twist)
            self.cmd_pub_generic.publish(twist)
            if not hasattr(self, '_publish_counter'):
                self._publish_counter = 0
            self._publish_counter += 1
            if self._publish_counter <= 20 or self._publish_counter % 10 == 0:
                self.get_logger().info(
                    f"cmd_vel pub #{self._publish_counter}: RECOVERY idx={self.current_idx}/{len(self.path)} "
                    f"target_idx={target_idx} pose=({px:.2f},{py:.2f},{yaw:.2f}) "
                    f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f} ang_err={ang_error:.2f} "
                    f"cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
                )
            return

        # angular control
        twist.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.angular_gain * ang_error),
        )

        # linear control
        if abs(ang_error) > 1.2:
            # Extreme misalignment: turn in place to avoid pushing into walls
            twist.linear.x = 0.0
        elif abs(ang_error) > self.ang_slowdown_threshold:
            # Strong misalignment: allow a small forward motion to avoid wall lock
            twist.linear.x = min(self.min_linear_speed, 0.12)
        else:
            v = self.linear_gain * dist * max(0.0, math.cos(ang_error))
            v = max(self.min_linear_speed, v) if dist > self.goal_tolerance else 0.0
            twist.linear.x = max(0.0, min(self.max_linear_speed, v))

        self.cmd_pub_model.publish(twist)
        self.cmd_pub_generic.publish(twist)
        self._last_cmd_linear = twist.linear.x
        if not hasattr(self, '_publish_counter'):
            self._publish_counter = 0
        self._publish_counter += 1
        if self._publish_counter <= 20 or self._publish_counter % 10 == 0:
            self.get_logger().info(
                f"cmd_vel pub #{self._publish_counter}: idx={self.current_idx}/{len(self.path)} "
                f"target_idx={target_idx} pose=({px:.2f},{py:.2f},{yaw:.2f}) "
                f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f} ang_err={ang_error:.2f} "
                f"cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
            )

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _nearest_index(self, x, y):
        if not self.path:
            return None
        best_i = None
        best_d = float('inf')
        # Search within a reasonable window ahead of current index
        start_i = max(0, self.current_idx - 5)
        for i in range(start_i, len(self.path)):
            px, py = self.path[i]
            d = (px - x) * (px - x) + (py - y) * (py - y)
            if d < best_d:
                best_d = d
                best_i = i
            # Early exit if squared distance starts increasing a lot
            if best_i is not None and i - best_i > 30:
                break
        return best_i


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
