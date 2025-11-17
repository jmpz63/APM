#!/usr/bin/env python3
"""FollowJointTrajectory -> JointTrajectory topic adapter.

This lets MoveIt execute plans by providing a simple action server that
immediately republishes the goal trajectory onto /joint_trajectory (or a
configured topic) which the existing trajectory_bridge already consumes.

Limitations:
 - No timing enforcement; assumes downstream bridge will pace execution.
 - Marks goal succeeded immediately after publishing (optimistic mode).
 - Does not provide feedback updates.

Future improvements:
 - Emit feedback at segment boundaries.
 - Validate joint names / reorder.
 - Optionally block until bridge reports completion.
"""
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class FollowJointTrajectoryAdapter(Node):
    def __init__(self):
        super().__init__('follow_joint_trajectory_adapter')
        self.declare_parameter('publish_topic', '/joint_trajectory')
        self.declare_parameter('accept_partial', False)
        # MoveIt discovery expects server at /<controller_name>/<action_ns>
        self.declare_parameter('controller_name', 'manipulator_controller')
        self.declare_parameter('action_ns', 'follow_joint_trajectory')
        # Execution timing behavior
        self.declare_parameter('wait_for_execution', True)
        self.declare_parameter('feedback_rate_hz', 2.0)
        self._pub_topic = self.get_parameter('publish_topic').value
        self._controller_name = self.get_parameter('controller_name').value
        self._action_ns = self.get_parameter('action_ns').value
        self._wait_for_execution = self.get_parameter('wait_for_execution').value
        self._feedback_rate_hz = self.get_parameter('feedback_rate_hz').value
        full_action_name = f"{self._controller_name}/{self._action_ns}".strip('/')
        self._pub = self.create_publisher(JointTrajectory, self._pub_topic, 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            full_action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            handle_accepted_callback=self._handle_accepted
        )
        self.get_logger().info(
            f"FollowJointTrajectory adapter active. Action server: /{full_action_name} publishing to {self._pub_topic}"
        )

    def goal_cb(self, goal_request: FollowJointTrajectory.Goal):
        # Basic validation: ensure points exist
        if not goal_request.trajectory.points:
            self.get_logger().warn('Rejected empty trajectory goal')
            return GoalResponse.REJECT
        self.get_logger().info(
            f"Accepted goal with {len(goal_request.trajectory.points)} points for joints {goal_request.trajectory.joint_names}"
        )
        return GoalResponse.ACCEPT

    # Removed custom handle_accepted to allow default execution path.

    def cancel_cb(self, goal_handle):  # pragma: no cover
        self.get_logger().info('Cancel request received (not implemented)')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):  # pragma: no cover
        try:
            goal = goal_handle.request
            npts = len(goal.trajectory.points)
            self.get_logger().info(
                f"EXECUTE_CB_START pts={npts} topic={self._pub_topic}"  # marker
            )
            if npts == 0:
                self.get_logger().warn('Execute received empty trajectory; aborting goal')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result
            
            # Publish trajectory to bridge
            self._pub.publish(goal.trajectory)
            self.get_logger().info(
                f"Republished trajectory pts={npts} joints={goal.trajectory.joint_names} -> {self._pub_topic}"
            )
            
            # Wait for execution if configured to do so
            if self._wait_for_execution and npts > 0:
                last_point = goal.trajectory.points[-1]
                expected_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
                self.get_logger().info(f"Waiting for trajectory execution (expected: {expected_duration:.2f}s)")
                
                # Provide periodic feedback during execution
                feedback_msg = FollowJointTrajectory.Feedback()
                feedback_msg.joint_names = goal.trajectory.joint_names
                
                # Wait for trajectory duration with periodic feedback
                import time
                feedback_interval = 1.0 / self._feedback_rate_hz if self._feedback_rate_hz > 0 else 0.5
                elapsed_time = 0.0
                
                while elapsed_time < expected_duration:
                    sleep_time = min(feedback_interval, expected_duration - elapsed_time)
                    time.sleep(sleep_time)
                    elapsed_time += sleep_time
                    
                    # Send progress feedback
                    progress = min(elapsed_time / expected_duration, 1.0)
                    feedback_msg.actual.time_from_start.sec = int(elapsed_time)
                    feedback_msg.actual.time_from_start.nanosec = int((elapsed_time - int(elapsed_time)) * 1e9)
                    goal_handle.publish_feedback(feedback_msg)
                    
                    self.get_logger().debug(f"Execution progress: {progress*100:.1f}%")
                
                # Add small buffer for hardware settling
                time.sleep(0.5)
            else:
                self.get_logger().info("Optimistic mode: returning success immediately")
            
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('EXECUTE_CB_DONE success - trajectory execution completed')
            return result
        except Exception as e:
            self.get_logger().error(f'Execute callback error: {e}')
            try:
                goal_handle.abort()
            except Exception:
                pass
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

    def _handle_accepted(self, goal_handle):  # pragma: no cover
        # Use default handling for synchronous execute callback
        self.get_logger().debug('Handle accepted invoked; using default execution')
        goal_handle.execute()


def main():  # pragma: no cover
    rclpy.init()
    node = FollowJointTrajectoryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
