#!/usr/bin/env python3
"""
Working MoveIt Commands - Simplified Version
Based on the successful test_execution_feedback.py approach
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math

class WorkingMoveItControl(Node):
    def __init__(self):
        super().__init__('working_moveit_control')
        # Use the EXACT same action client as the working test
        self._ac = ActionClient(self, FollowJointTrajectory, '/manipulator_controller/follow_joint_trajectory')

    def move_joint_degrees(self, degrees, duration_sec=3):
        """Move joint1 to specified degrees - EXACTLY like the working test"""
        if not self._ac.wait_for_server(timeout_sec=5.0):
            print('❌ Action server not available')
            return False
        
        radians = math.radians(degrees)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1']
        
        # Single point trajectory (simpler than the test's 2-point)
        point = JointTrajectoryPoint()
        point.positions = [radians]
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0
        
        goal.trajectory.points = [point]
        
        print(f'🎯 Moving joint1 to {degrees}° ({radians:.3f} rad)')
        
        # Send goal
        send_future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('❌ Goal rejected')
            return False
            
        print('✅ Goal accepted, waiting for completion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            print(f'✅ Successfully moved to {degrees}°!')
            return True
        else:
            print(f'❌ Movement failed with error {result.error_code}')
            return False

def main():
    rclpy.init()
    controller = WorkingMoveItControl()
    
    print("\n" + "="*50)
    print("WORKING MOVEIT CONTROL")
    print("="*50)
    print("Based on your successful test_execution_feedback.py")
    print("="*50)
    
    # Test movements
    movements = [
        (10, "Small positive movement"),
        (-10, "Small negative movement"), 
        (20, "Medium positive movement"),
        (0, "Return to zero")
    ]
    
    try:
        for degrees, description in movements:
            print(f"\n🔄 {description}")
            input(f"Press Enter to move to {degrees}°...")
            
            success = controller.move_joint_degrees(degrees)
            
            if not success:
                print("❌ Movement failed, stopping sequence")
                break
                
            input("Press Enter to continue to next movement...")
            
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()