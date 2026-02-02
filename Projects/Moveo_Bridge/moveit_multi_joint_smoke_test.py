#!/usr/bin/env python3
"""Multi-joint MoveIt2 smoke test (headless).

Goal: prove that MoveIt can plan (and optionally execute) a joint-space target
for the 5-DOF Moveo configuration.

Usage (Terminal A):
  source /opt/ros/humble/setup.bash
  cd /home/arm1/APM/Projects/Moveo_Bridge
  source install/setup.bash
  ros2 launch moveo_moveit_config demo.launch.py use_rviz:=false

Usage (Terminal B):
  source /opt/ros/humble/setup.bash
  cd /home/arm1/APM/Projects/Moveo_Bridge
  source install/setup.bash
  python3 moveit_multi_joint_smoke_test.py --plan-only

If you have a live fjt_adapter bridge and want to execute on hardware,
run without --plan-only (NOT recommended until you confirm limits/safety).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan


@dataclass(frozen=True)
class JointTarget:
    name: str
    position: float


class MoveItMotionPlanClient(Node):
    def __init__(self, group_name: str):
        super().__init__('moveit_motion_plan_client')
        self._group_name = group_name
        self._client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

    def wait_ready(self, timeout_sec: float = 10.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def plan(self, targets: List[JointTarget], allowed_planning_time: float = 5.0) -> GetMotionPlan.Response:
        req = GetMotionPlan.Request()

        mpr = MotionPlanRequest()
        mpr.group_name = self._group_name
        mpr.planner_id = 'RRTConnect'
        mpr.allowed_planning_time = float(allowed_planning_time)
        mpr.num_planning_attempts = 1

        # Start state: keep empty; move_group will populate from CurrentStateMonitor
        mpr.start_state = RobotState()

        goal_constraints = Constraints()
        goal_constraints.name = 'joint_goal'

        for jt in targets:
            jc = JointConstraint()
            jc.joint_name = jt.name
            jc.position = float(jt.position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        mpr.goal_constraints = [goal_constraints]
        req.motion_plan_request = mpr

        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument('--group', default='manipulator', help='MoveIt planning group name')
    p.add_argument('--plan-only', action='store_true', help='Only plan, do not execute (safe default)')
    # Keep targets very small by default.
    p.add_argument('--j1', type=float, default=0.0, help='joint1 target (rad)')
    p.add_argument('--j2', type=float, default=0.2, help='joint2 target (rad)')
    p.add_argument('--j3', type=float, default=-0.2, help='joint3 target (rad)')
    p.add_argument('--j4', type=float, default=0.2, help='joint4 target (rad)')
    p.add_argument('--j5', type=float, default=0.0, help='joint5 target (rad)')
    return p.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = MoveItMotionPlanClient(group_name=args.group)

    try:
        if not node.wait_ready(timeout_sec=10.0):
            node.get_logger().error('MoveIt planning service /plan_kinematic_path not available')
            node.get_logger().error('Is demo.launch.py running? (use_rviz:=false is fine)')
            return 2

        targets = [
            JointTarget('joint1', args.j1),
            JointTarget('joint2', args.j2),
            JointTarget('joint3', args.j3),
            JointTarget('joint4', args.j4),
            JointTarget('joint5', args.j5),
        ]

        node.get_logger().info('Requesting plan for 5-joint target...')
        res = node.plan(targets=targets, allowed_planning_time=5.0)

        if res is None:
            node.get_logger().error('No response from planning service')
            return 3

        err = res.motion_plan_response.error_code.val
        if err != 1:
            node.get_logger().error(f'Planning failed with MoveItErrorCodes={err}')
            return 4

        traj = res.motion_plan_response.trajectory
        points = 0
        if traj.joint_trajectory.points:
            points = len(traj.joint_trajectory.points)

        node.get_logger().info(f'Planning OK. Trajectory points: {points}')

        if args.plan_only:
            node.get_logger().info('Plan-only mode: not executing.')
            return 0

        node.get_logger().warning('Execution not implemented in this smoke test (use existing FJT scripts/bridge).')
        node.get_logger().warning('If you want, I can wire execution via ExecuteTrajectoryAction safely.')
        return 0

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
