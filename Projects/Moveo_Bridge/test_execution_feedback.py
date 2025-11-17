#!/usr/bin/env python3
"""
Test script to verify improved FollowJointTrajectory execution with proper feedback.
This demonstrates the difference between optimistic and realistic execution behavior.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class ImprovedTestClient(Node):
    def __init__(self):
        super().__init__('improved_fjt_test_client')
        self._ac = ActionClient(self, FollowJointTrajectory, '/manipulator_controller/follow_joint_trajectory')
        self.sent = False

    def send_trajectory(self, duration_sec=4.0):
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowJointTrajectory action server not available!')
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1']
        
        # Create a trajectory with specified duration
        p1 = JointTrajectoryPoint()
        p1.positions = [0.5]  # Move to 0.5 radians (~28.6 degrees)
        p1.time_from_start.sec = int(duration_sec / 2)
        p1.time_from_start.nanosec = int((duration_sec / 2 - int(duration_sec / 2)) * 1e9)
        
        p2 = JointTrajectoryPoint()
        p2.positions = [0.0]  # Return to 0
        p2.time_from_start.sec = int(duration_sec)
        p2.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        goal.trajectory.points = [p1, p2]
        
        self.get_logger().info(f'Sending trajectory goal (duration: {duration_sec}s)')
        self.get_logger().info('Watch for periodic feedback messages during execution...')
        
        send_future = self._ac.send_goal_async(goal, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        timestamp = feedback.actual.time_from_start.sec + feedback.actual.time_from_start.nanosec * 1e-9
        self.get_logger().info(f'Execution feedback: t={timestamp:.2f}s')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Execution completed! Result: error_code={result.error_code}')
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ SUCCESS: Trajectory execution reported as completed')
        else:
            self.get_logger().warn(f'⚠️  Non-success result: {result.error_code}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ImprovedTestClient()
    
    print("\n" + "="*60)
    print("TESTING IMPROVED FOLLOWJOINTTRAJECTORY EXECUTION")
    print("="*60)
    print("This test demonstrates:")
    print("1. Proper execution timing (waits for trajectory duration)")
    print("2. Periodic feedback during execution")
    print("3. Realistic success reporting")
    print("="*60 + "\n")
    
    if node.send_trajectory(duration_sec=6.0):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()