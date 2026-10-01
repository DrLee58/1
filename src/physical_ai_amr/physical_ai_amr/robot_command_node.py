import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty


class RobotCommandNode(Node):
    """Patrols reactively by default; hands control to Nav2 for Nav2 goals.

    Publishing a PoseStamped on /go_to_pose pauses the reactive patrol and
    sends it to Nav2's navigate_to_pose action (see navigation.launch.py).
    Once that navigation finishes (success or failure), patrol resumes.

    Wandering past the patrol radius also triggers a Nav2 goal back to the
    origin, rather than the fixed backup+turn used for obstacles: a blind
    turn has no guarantee of aiming back inward, and in practice it can
    settle into a stable loop that oscillates right on the boundary forever.

    Publishing an Empty on /start_waypoint_tour switches to cycling through
    WAYPOINTS via Nav2 forever, instead of reactive patrolling; /stop_waypoint_tour
    switches back.
    """

    FORWARD_SPEED = 0.3
    BACKUP_SPEED = -0.15
    TURN_SPEED = 0.6
    AVOID_DURATION = rclpy.duration.Duration(seconds=2.0)

    # Matches the obstacle_box pose in worlds/empty_world.sdf.
    OBSTACLE_POSITION = (2.0, 0.0)
    SAFE_DISTANCE = 1.0

    # Keep the robot patrolling near the origin instead of wandering off forever.
    PATROL_RADIUS = 4.0

    # If Nav2 can't complete a goal (e.g. the robot is outside the known map),
    # it can fail instantly - without this, a failed patrol-edge return would
    # retry every odom message and spam the action server forever.
    NAV_RETRY_COOLDOWN = rclpy.duration.Duration(seconds=3.0)

    # A loop around the obstacle_box, staying inside maps/patrol_world.yaml's
    # [-6, 6] extent.
    WAYPOINTS = [
        (4.0, 4.0),
        (4.0, -4.0),
        (-4.0, -4.0),
        (-4.0, 4.0),
    ]

    def __init__(self):
        super().__init__('robot_command_node')

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/go_to_pose',
            self.go_to_pose_callback,
            10
        )

        self.start_tour_subscription = self.create_subscription(
            Empty,
            '/start_waypoint_tour',
            self.start_tour_callback,
            10
        )

        self.stop_tour_subscription = self.create_subscription(
            Empty,
            '/stop_waypoint_tour',
            self.stop_tour_callback,
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.avoiding_until = None
        self.navigating = False
        self.nav_retry_until = None
        self.touring = False
        self.waypoint_index = 0

        self.timer = self.create_timer(
            0.1,
            self.publish_command
        )

        self.get_logger().info('Robot command node started')

    def odom_callback(self, msg):
        if self.navigating:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        near_obstacle = math.hypot(
            x - self.OBSTACLE_POSITION[0], y - self.OBSTACLE_POSITION[1]
        ) < self.SAFE_DISTANCE
        if near_obstacle and self.avoiding_until is None:
            self.avoiding_until = self.get_clock().now() + self.AVOID_DURATION
            self.get_logger().info('Obstacle nearby, backing off and turning')
            return

        if self.touring:
            return

        if math.hypot(x, y) > self.PATROL_RADIUS:
            if self.nav_retry_until is not None and self.get_clock().now() < self.nav_retry_until:
                return
            self.get_logger().info('Patrol edge reached, returning to origin via Nav2')
            origin = PoseStamped()
            origin.header.frame_id = 'map'
            origin.pose.orientation.w = 1.0
            self.send_nav_goal(origin)

    def go_to_pose_callback(self, msg):
        self.touring = False
        self.send_nav_goal(msg)

    def start_tour_callback(self, _msg):
        self.get_logger().info('Starting waypoint tour')
        self.touring = True
        self.waypoint_index = 0
        self.nav_retry_until = None
        self._send_current_waypoint()

    def stop_tour_callback(self, _msg):
        self.get_logger().info('Stopping waypoint tour, resuming patrol')
        self.touring = False

    def _send_current_waypoint(self):
        x, y = self.WAYPOINTS[self.waypoint_index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.send_nav_goal(pose)

    def send_nav_goal(self, pose_stamped):
        if self.navigating:
            self.get_logger().warn('Already navigating to a goal, ignoring new request')
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        self.navigating = True
        self.avoiding_until = None
        self.get_logger().info('Pausing patrol, handing control to Nav2')

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal, resuming patrol')
            self.navigating = False
            self._on_nav_failure()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.navigating = False

        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Nav2 goal did not succeed (status={future.result().status}), '
                f'resuming patrol after a cooldown'
            )
            self._on_nav_failure()
            return

        self.nav_retry_until = None
        if self.touring:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.WAYPOINTS)
            self.get_logger().info(f'Waypoint reached, heading to waypoint {self.waypoint_index}')
            self._send_current_waypoint()
        else:
            self.get_logger().info('Nav2 goal finished, resuming patrol')

    def _on_nav_failure(self):
        # Don't just drive straight ahead while waiting to retry - that would
        # carry the robot further from anywhere Nav2 can reach. Fall back to
        # the same local backup+turn used for obstacles.
        self.nav_retry_until = self.get_clock().now() + self.NAV_RETRY_COOLDOWN
        self.avoiding_until = self.get_clock().now() + self.AVOID_DURATION

    def publish_command(self):
        if self.navigating:
            return

        msg = Twist()

        if self.avoiding_until is not None:
            if self.get_clock().now() >= self.avoiding_until:
                self.avoiding_until = None
            else:
                msg.linear.x = self.BACKUP_SPEED
                msg.angular.z = self.TURN_SPEED
                self.publisher_.publish(msg)
                return

        if self.touring:
            if self.nav_retry_until is None or self.get_clock().now() >= self.nav_retry_until:
                self._send_current_waypoint()
            return

        msg.linear.x = self.FORWARD_SPEED
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RobotCommandNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
