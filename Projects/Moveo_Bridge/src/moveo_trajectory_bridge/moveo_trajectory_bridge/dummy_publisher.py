import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DummyTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('dummy_trajectory_publisher')
        self.declare_parameter('publish_rate', 0.5)  # Hz per full trajectory cycle
        self.declare_parameter('joint_names', ['joint1','joint2','joint3','joint4','joint5','joint6'])
        self.declare_parameter('topic', '/joint_trajectory')
        self.declare_parameter('one_shot', False)
        self.declare_parameter('segments', 20)
        self.declare_parameter('duration', 2.0)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._topic = self.get_parameter('topic').value
        self._one_shot = bool(self.get_parameter('one_shot').value)
        self._segments = int(self.get_parameter('segments').value)
        self._duration = float(self.get_parameter('duration').value)
        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)
        if self._one_shot:
            # publish once after short delay
            self._oneshot_timer = self.create_timer(0.2, self._publish_once)
        else:
            period = 1.0/self._rate if self._rate > 0 else 5.0
            self._timer = self.create_timer(period, self._publish)
        self._phase = 0.0
        mode = 'ONE-SHOT' if self._one_shot else f'REPEATING @ {self._rate} Hz'
        self.get_logger().info(f"Publishing dummy JointTrajectory to {self._topic} ({mode})")

    def _build_traj(self):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        for i in range(self._segments+1):
            frac = i/self._segments
            pt = JointTrajectoryPoint()
            # Simple sinusoidal sweep for each joint with phase offsets.
            pt.positions = [0.5*math.sin(self._phase + idx*0.3 + frac*math.pi) for idx in range(len(self._joint_names))]
            t = frac * self._duration
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t))*1e9)
            traj.points.append(pt)
        return traj

    def _publish(self):
        traj = self._build_traj()
        self._pub.publish(traj)
        self._phase += 0.2

    def _publish_once(self):
        if getattr(self, '_oneshot_timer', None) is None:
            return  # already fired
        traj = self._build_traj()
        self._pub.publish(traj)
        # Cancel timer to prevent any edge race firing again
        try:
            self._oneshot_timer.cancel()
        except Exception:
            pass
        self._oneshot_timer = None
        self.get_logger().info("One-shot trajectory published; shutting down node.")
        # Allow a brief delay for subscribers to process before shutdown
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = DummyTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
