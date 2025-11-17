import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class MoveItDisplayRelay(Node):
    """Relay MoveIt's DisplayTrajectory planned path to a plain JointTrajectory topic.

    Simplest integration path: whenever a new DisplayTrajectory arrives, take the first
    robot_trajectory, extract its joint_trajectory and publish it to /joint_trajectory (configurable).

    Optional narrowing:
      - only_first: if true, only publish the first time stamp sequence (default True)
      - joints_subset: list of joints to keep (others zeroed). If empty, keep all.
      - position_unit: 'radians' or 'degrees' for output (bridge expects radians by default).
    """
    def __init__(self):
        super().__init__('moveit_display_relay')
        self.declare_parameter('display_topic', '/move_group/display_planned_path')
        self.declare_parameter('output_topic', '/joint_trajectory')
        self.declare_parameter('only_first', True)
        # Use an explicitly typed empty string array default (avoid [] ambiguity in rclpy)
        self.declare_parameter('joints_subset', [''])
        self.declare_parameter('position_unit', 'radians')
        self.declare_parameter('deg_round', 5)
        self._display_topic = self.get_parameter('display_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._only_first = bool(self.get_parameter('only_first').value)
        raw_subset = self.get_parameter('joints_subset').value
        if raw_subset and isinstance(raw_subset, list):
            # Filter out empty placeholder strings
            self._subset = [s for s in raw_subset if isinstance(s, str) and s]
        else:
            self._subset = []
        self._unit = self.get_parameter('position_unit').value
        self._deg_round = int(self.get_parameter('deg_round').value)
        self._pub = self.create_publisher(JointTrajectory, self._output_topic, 10)
        qos = QoSProfile(depth=5)
        self._sub = self.create_subscription(DisplayTrajectory, self._display_topic, self._cb, qos)
        self.get_logger().info(f"Relaying DisplayTrajectory '{self._display_topic}' -> JointTrajectory '{self._output_topic}'")

    def _cb(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return
        robot_traj = msg.trajectory[0]  # first trajectory
        jt = robot_traj.joint_trajectory
        if not jt.points:
            return
        out = JointTrajectory()
        if self._subset:
            # filter joints
            # build mapping existing index; if missing joint add zero
            indices = []
            for name in self._subset:
                if name in jt.joint_names:
                    indices.append(jt.joint_names.index(name))
                else:
                    indices.append(-1)
            out.joint_names = list(self._subset)
            for pt in jt.points:
                new_pt = JointTrajectoryPoint()
                for idx in indices:
                    if idx >= 0:
                        val = pt.positions[idx]
                    else:
                        val = 0.0
                    if self._unit == 'degrees':
                        val = round(math.degrees(val), self._deg_round)
                    new_pt.positions.append(val)
                new_pt.time_from_start = pt.time_from_start
                out.points.append(new_pt)
        else:
            out.joint_names = list(jt.joint_names)
            for pt in jt.points:
                new_pt = JointTrajectoryPoint()
                for val in pt.positions:
                    if self._unit == 'degrees':
                        val = round(math.degrees(val), self._deg_round)
                    new_pt.positions.append(val)
                new_pt.time_from_start = pt.time_from_start
                out.points.append(new_pt)
        self._pub.publish(out)
        self.get_logger().info(
            f"Published JointTrajectory points={len(out.points)} joints={len(out.joint_names)} subset={'yes' if self._subset else 'no'}")
        if self._only_first:
            # prevent republishing repeated display updates until user re-plans
            self.destroy_subscription(self._sub)
            self.get_logger().info('only_first=True: unsubscribed after first publish')


def main():  # pragma: no cover
    rclpy.init()
    node = MoveItDisplayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  # pragma: no cover
    main()
