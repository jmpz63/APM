#!/usr/bin/env python3
"""
Precise MoveIt Control Examples - Fine-Tuned Joint Control
Since test_execution_feedback.py worked, here are more controlled examples.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import math

class PreciseMoveitControl(Node):
    def __init__(self):
        super().__init__('precise_moveit_control')
        self._ac = ActionClient(self, FollowJointTrajectory, '/manipulator_controller/follow_joint_trajectory')
        self.get_logger().info("Precise MoveIt Control ready")

    def move_joint_precise(self, joint_angle_degrees, duration_sec=3.0, joint_name='joint1'):
        """Move a single joint to precise angle in degrees"""
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
        
        angle_rad = math.radians(joint_angle_degrees)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
        
        # Create smooth trajectory with intermediate point
        p1 = JointTrajectoryPoint()
        p1.positions = [angle_rad / 2]  # Halfway point
        p1.time_from_start.sec = int(duration_sec / 2)
        p1.time_from_start.nanosec = int(((duration_sec / 2) % 1) * 1e9)
        
        p2 = JointTrajectoryPoint()
        p2.positions = [angle_rad]  # Final position
        p2.time_from_start.sec = int(duration_sec)
        p2.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        goal.trajectory.points = [p1, p2]
        
        self.get_logger().info(f'Moving {joint_name} to {joint_angle_degrees}° ({angle_rad:.3f} rad) over {duration_sec}s')
        
        # Send goal and wait
        send_future = self._ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        
        if success:
            self.get_logger().info(f'✅ Successfully moved to {joint_angle_degrees}°')
        else:
            self.get_logger().error(f'❌ Motion failed with error {result.error_code}')
            
        return success

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.actual.positions:
            actual_deg = math.degrees(feedback.actual.positions[0])
            timestamp = feedback.actual.time_from_start.sec + feedback.actual.time_from_start.nanosec * 1e-9
            self.get_logger().info(f'  Progress: {actual_deg:.1f}° at t={timestamp:.1f}s')

    def move_sequence(self, angles_degrees, joint_name='joint1', pause_sec=1.0):
        """Execute a sequence of joint movements"""
        self.get_logger().info(f"Starting movement sequence: {angles_degrees}°")
        
        for i, angle in enumerate(angles_degrees):
            self.get_logger().info(f"Step {i+1}/{len(angles_degrees)}: Moving to {angle}°")
            
            if not self.move_joint_precise(angle, duration_sec=2.0, joint_name=joint_name):
                self.get_logger().error(f"Sequence failed at step {i+1}")
                return False
                
            if pause_sec > 0 and i < len(angles_degrees) - 1:
                self.get_logger().info(f"Pausing {pause_sec}s before next move...")
                time.sleep(pause_sec)
        
        self.get_logger().info("✅ Sequence completed!")
        return True

def main():
    rclpy.init()
    controller = PreciseMoveitControl()
    
    print("\n" + "="*60)
    print("PRECISE MOVEIT JOINT CONTROL")
    print("="*60)
    print("Choose an example:")
    print("1. Small movements (±5°)")
    print("2. Medium movements (±15°)")  
    print("3. Larger movements (±30°)")
    print("4. Movement sequence")
    print("5. Return to zero")
    print("="*60)
    
    try:
        choice = input("Enter choice (1-5): ").strip()
        
        if choice == "1":
            print("Testing small ±5° movements...")
            controller.move_sequence([5, -5, 0])
            
        elif choice == "2":
            print("Testing medium ±15° movements...")
            controller.move_sequence([15, -15, 0])
            
        elif choice == "3":
            print("Testing larger ±30° movements...")
            controller.move_sequence([30, -30, 0])
            
        elif choice == "4":
            print("Testing smooth sequence...")
            controller.move_sequence([10, 20, 15, 5, -10, 0], pause_sec=0.5)
            
        elif choice == "5":
            print("Returning to zero position...")
            controller.move_joint_precise(0, duration_sec=3.0)
            
        else:
            print("Invalid choice. Testing 10° movement...")
            controller.move_joint_precise(10)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()