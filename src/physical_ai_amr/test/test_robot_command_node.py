from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration

from physical_ai_amr.robot_command_node import RobotCommandNode


def make_odom(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    return msg


@pytest.fixture
def node():
    rclpy.init()
    n = RobotCommandNode()
    n.publisher_.publish = MagicMock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


class TestPatrolAvoidance:

    def test_forward_when_clear(self, node):
        node.odom_callback(make_odom(0.0, 0.0))
        node.publish_command()

        msg = node.publisher_.publish.call_args.args[0]
        assert msg.linear.x == RobotCommandNode.FORWARD_SPEED
        assert msg.angular.z == 0.0

    def test_triggers_avoidance_near_obstacle(self, node):
        obstacle_x, obstacle_y = RobotCommandNode.OBSTACLE_POSITION
        node.odom_callback(make_odom(obstacle_x - 0.5, obstacle_y))

        assert node.avoiding_until is not None

    def test_no_avoidance_just_outside_safe_distance(self, node):
        obstacle_x, obstacle_y = RobotCommandNode.OBSTACLE_POSITION
        node.odom_callback(
            make_odom(obstacle_x - RobotCommandNode.SAFE_DISTANCE - 0.1, obstacle_y)
        )

        assert node.avoiding_until is None

    def test_past_patrol_radius_sends_nav2_goal_to_origin(self, node):
        fake_future = MagicMock()

        with patch.object(node.nav_client, 'wait_for_server', return_value=True), \
                patch.object(
                    node.nav_client, 'send_goal_async', return_value=fake_future
                ) as send_mock:
            node.odom_callback(make_odom(RobotCommandNode.PATROL_RADIUS + 1.0, 0.0))

        assert node.navigating is True
        goal = send_mock.call_args.args[0]
        assert goal.pose.pose.position.x == 0.0
        assert goal.pose.pose.position.y == 0.0

    def test_no_new_avoidance_while_already_avoiding(self, node):
        node.avoiding_until = node.get_clock().now() + Duration(seconds=5.0)
        first_deadline = node.avoiding_until

        obstacle_x, obstacle_y = RobotCommandNode.OBSTACLE_POSITION
        node.odom_callback(make_odom(obstacle_x, obstacle_y))

        assert node.avoiding_until == first_deadline

    def test_avoidance_publishes_backup_and_turn(self, node):
        node.avoiding_until = node.get_clock().now() + Duration(seconds=1.0)

        node.publish_command()

        msg = node.publisher_.publish.call_args.args[0]
        assert msg.linear.x == RobotCommandNode.BACKUP_SPEED
        assert msg.angular.z == RobotCommandNode.TURN_SPEED

    def test_avoidance_expires_and_resumes_forward(self, node):
        node.avoiding_until = node.get_clock().now() - Duration(seconds=1.0)

        node.publish_command()

        assert node.avoiding_until is None
        msg = node.publisher_.publish.call_args.args[0]
        assert msg.linear.x == RobotCommandNode.FORWARD_SPEED


class TestNav2Handoff:

    def test_odom_ignored_while_navigating(self, node):
        node.navigating = True
        obstacle_x, obstacle_y = RobotCommandNode.OBSTACLE_POSITION

        node.odom_callback(make_odom(obstacle_x, obstacle_y))

        assert node.avoiding_until is None

    def test_publish_command_skipped_while_navigating(self, node):
        node.navigating = True

        node.publish_command()

        node.publisher_.publish.assert_not_called()

    def test_go_to_pose_ignored_when_already_navigating(self, node):
        node.navigating = True

        with patch.object(node.nav_client, 'wait_for_server') as wait_mock:
            node.go_to_pose_callback(PoseStamped())

        wait_mock.assert_not_called()

    def test_go_to_pose_handles_unavailable_server(self, node):
        with patch.object(node.nav_client, 'wait_for_server', return_value=False):
            node.go_to_pose_callback(PoseStamped())

        assert node.navigating is False

    def test_go_to_pose_sends_goal_and_pauses_patrol(self, node):
        node.avoiding_until = node.get_clock().now() + Duration(seconds=5.0)
        fake_future = MagicMock()

        with patch.object(node.nav_client, 'wait_for_server', return_value=True), \
                patch.object(
                    node.nav_client, 'send_goal_async', return_value=fake_future
                ) as send_mock:
            node.go_to_pose_callback(PoseStamped())

        assert node.navigating is True
        assert node.avoiding_until is None
        send_mock.assert_called_once()
        fake_future.add_done_callback.assert_called_once_with(
            node.nav_goal_response_callback
        )

    def test_nav_goal_rejected_resumes_patrol(self, node):
        node.navigating = True
        future = MagicMock()
        future.result.return_value = MagicMock(accepted=False)

        node.nav_goal_response_callback(future)

        assert node.navigating is False

    def test_nav_goal_accepted_awaits_result(self, node):
        node.navigating = True
        goal_handle = MagicMock(accepted=True)
        future = MagicMock()
        future.result.return_value = goal_handle

        node.nav_goal_response_callback(future)

        goal_handle.get_result_async.assert_called_once()
        assert node.navigating is True  # still true until the result callback fires

    def test_go_to_pose_preserves_requested_orientation(self, node):
        msg = PoseStamped()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.orientation.z = 0.7071
        msg.pose.orientation.w = 0.7071
        fake_future = MagicMock()

        with patch.object(node.nav_client, 'wait_for_server', return_value=True), \
                patch.object(
                    node.nav_client, 'send_goal_async', return_value=fake_future
                ) as send_mock:
            node.go_to_pose_callback(msg)

        goal = send_mock.call_args.args[0]
        assert goal.pose.pose.orientation.z == 0.7071
        assert goal.pose.pose.orientation.w == 0.7071

    def test_nav_result_resumes_patrol(self, node):
        node.navigating = True

        node.nav_result_callback(MagicMock())

        assert node.navigating is False
