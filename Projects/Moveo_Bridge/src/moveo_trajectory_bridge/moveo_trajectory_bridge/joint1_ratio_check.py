import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Joint1RatioCheck(Node):
    """Publishes a single 36° (or configurable) move for Joint1 to verify 1 motor rev.

    Because gear_ratio is 10:1 and rotation_distance is 360 (deg units),
    a 36° joint move corresponds to one full motor revolution.
    """
    def __init__(self):
        super().__init__('joint1_ratio_check')
        self.declare_parameter('topic', '/joint_trajectory')
        self.declare_parameter('use_radians', True)
        self.declare_parameter('angle_deg', 36.0)
        self.declare_parameter('duration', 2.0)
        self._topic = self.get_parameter('topic').value
        self._use_radians = bool(self.get_parameter('use_radians').value)
        self._angle = float(self.get_parameter('angle_deg').value)
        self._duration = float(self.get_parameter('duration').value)
        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)
        self.create_timer(0.25, self._publish)
        self.get_logger().info(f"Joint1 ratio test scheduled: angle={self._angle} deg over {self._duration}s")

    def _publish(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
        # Start point (0)
        p0 = JointTrajectoryPoint()
        p0.positions = [0.0,0.0,0.0,0.0,0.0,0.0]
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        traj.points.append(p0)
        # Target point
        p1 = JointTrajectoryPoint()
        val = math.radians(self._angle) if self._use_radians else self._angle
        p1.positions = [val,0.0,0.0,0.0,0.0,0.0]
        p1.time_from_start.sec = int(self._duration)
        p1.time_from_start.nanosec = int((self._duration - int(self._duration))*1e9)
        traj.points.append(p1)
        self._pub.publish(traj)
        self.get_logger().info("Ratio verification trajectory published; shut down in 0.5s.")
        self.create_timer(0.5, lambda: rclpy.shutdown())


def main():  # pragma: no cover
    rclpy.init()
    node = Joint1RatioCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  # pragma: no cover
    main()
