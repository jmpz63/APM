import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Joint1TestPublisher(Node):
    """Publishes a simple trajectory exercising Joint1 only.

    Sequence (degrees intent, converted to radians if using radians):
      0° (center) -> +30° -> -30° -> 0° over uniform timing.
    """
    def __init__(self):
        super().__init__('joint1_test_publisher')
        self.declare_parameter('topic', '/joint_trajectory')
        self.declare_parameter('use_radians', True)
        self.declare_parameter('segment_time', 1.0)  # seconds per leg
        self.declare_parameter('one_shot', True)
        self._topic = self.get_parameter('topic').value
        self._use_radians = bool(self.get_parameter('use_radians').value)
        self._seg_time = float(self.get_parameter('segment_time').value)
        self._one_shot = bool(self.get_parameter('one_shot').value)
        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)
        # Fire once shortly after start
        delay = 0.25
        self.create_timer(delay, self._emit if self._one_shot else self._emit_repeat)
        self._repeat_timer = None
        self.get_logger().info(f"Joint1 test publisher ready (topic={self._topic})")

    def _build(self):
        # Desired joint1 positions in degrees
        seq_deg = [0.0, 30.0, -30.0, 0.0]
        pts = []
        t_accum = 0.0
        traj = JointTrajectory()
        traj.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        for d in seq_deg:
            pt = JointTrajectoryPoint()
            val = math.radians(d) if self._use_radians else d
            pt.positions = [val, 0.0, 0.0, 0.0, 0.0, 0.0]
            pt.time_from_start.sec = int(t_accum)
            pt.time_from_start.nanosec = int((t_accum - int(t_accum))*1e9)
            traj.points.append(pt)
            t_accum += self._seg_time
        return traj

    def _emit(self):
        traj = self._build()
        self._pub.publish(traj)
        self.get_logger().info("Joint1 test trajectory published.")
        if self._one_shot:
            # Optionally shutdown after short grace
            self.create_timer(0.5, lambda: rclpy.shutdown())

    def _emit_repeat(self):  # pragma: no cover
        traj = self._build()
        self._pub.publish(traj)
        self.get_logger().info("Joint1 test trajectory cycle published.")
        if self._repeat_timer is None:
            self._repeat_timer = self.create_timer(self._seg_time * 4.0 + 1.0, self._emit_repeat)

def main():  # pragma: no cover
    rclpy.init()
    node = Joint1TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  # pragma: no cover
    main()
