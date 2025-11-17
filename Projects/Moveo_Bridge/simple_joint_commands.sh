#!/bin/bash
# Simple Joint Movement Command# 1. Small 5° movement (2-point trajectory):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.044], time_from_start: {sec: 1, nanosec: 0}}, {positions: [0.087], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# 2. Medium 15° movement (2-point trajectory):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.131], time_from_start: {sec: 1, nanosec: 500000000}}, {positions: [0.262], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback

# 3. Return to zero (2-point trajectory):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.1], time_from_start: {sec: 1, nanosec: 0}}, {positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# 4. Large 30° movement (2-point trajectory):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.262], time_from_start: {sec: 2, nanosec: 0}}, {positions: [0.524], time_from_start: {sec: 4, nanosec: 0}}]}}" --feedback

# 5. Negative direction -20° (2-point trajectory):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [-0.175], time_from_start: {sec: 1, nanosec: 500000000}}, {positions: [-0.349], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback - Based on What Worked
# Since python3 test_execution_feedback.py worked, these should too

echo "=================================="
echo "SIMPLE JOINT MOVEMENT COMMANDS"
echo "=================================="
echo "These use the same working method as your test script"
echo ""

# Function to create trajectory command
create_joint_trajectory() {
    local angle_deg=$1
    local duration=$2
    local joint_name=${3:-"joint1"}
    
    # Convert degrees to radians
    local angle_rad=$(echo "scale=4; $angle_deg * 3.14159 / 180" | bc -l)
    
    echo "Moving $joint_name to $angle_deg degrees ($angle_rad rad) over ${duration}s..."
    
    # Create the ROS action command
    ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: ['$joint_name'],
            points: [
                {
                    positions: [$(echo "scale=4; $angle_rad / 2" | bc -l)],
                    time_from_start: {sec: $(( duration / 2 )), nanosec: 0}
                },
                {
                    positions: [$angle_rad], 
                    time_from_start: {sec: $duration, nanosec: 0}
                }
            ]
        }
    }" --feedback
}

echo "Available commands (copy and paste these):"
echo ""

echo "# 1. Small 5° movement:"
echo "ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: ['joint1'], points: [{positions: [0.087], time_from_start: {sec: 2, nanosec: 0}}]}}\" --feedback"
echo ""

echo "# 2. Medium 15° movement:"
echo "ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: ['joint1'], points: [{positions: [0.262], time_from_start: {sec: 3, nanosec: 0}}]}}\" --feedback"
echo ""

echo "# 3. Return to zero:" 
echo "ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: ['joint1'], points: [{positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}}\" --feedback"
echo ""

echo "# 4. Large 30° movement:"
echo "ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: ['joint1'], points: [{positions: [0.524], time_from_start: {sec: 4, nanosec: 0}}]}}\" --feedback"
echo ""

echo "# 5. Negative direction -20°:"
echo "ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: ['joint1'], points: [{positions: [-0.349], time_from_start: {sec: 3, nanosec: 0}}]}}\" --feedback"
echo ""

echo "=================================="
echo "ANGLE REFERENCE:"
echo "  5° = 0.087 rad"
echo " 10° = 0.175 rad" 
echo " 15° = 0.262 rad"
echo " 20° = 0.349 rad"
echo " 30° = 0.524 rad"
echo " 45° = 0.785 rad"
echo "=================================="
echo ""
echo "Usage: Copy any command above and paste into terminal"
echo "The arm should move to that position!"
echo ""