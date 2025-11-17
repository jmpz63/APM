import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class JointStateShim(Node):
    """Publishes joint_states by mirroring the most recent JointTrajectory point.
    Fallback: publishes zeros if no trajectory seen.
    This is a stop-gap until real feedback is integrated from hardware.
    """
    def __init__(self):
        super().__init__('joint_state_shim')
        self.declare_parameter('trajectory_topic', '/joint_trajectory')
        self.declare_parameter('publish_rate', 20.0)
        self._traj_topic = self.get_parameter('trajectory_topic').value
        self._rate = float(self.get_parameter('publish_rate').value)
        self._last_names: List[str] = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self._last_positions: List[float] = [0.0]*6
        self._last_stamp = self.get_clock().now()
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._sub = self.create_subscription(JointTrajectory, self._traj_topic, self._traj_cb, 10)
        period = 1.0/self._rate if self._rate > 0 else 0.05
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"JointState shim active (listening to {self._traj_topic})")

    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        # Use the last point (target goal) for current desired state approximation.
        pt = msg.points[-1]
        if msg.joint_names:
            self._last_names = list(msg.joint_names)
        if len(pt.positions) == len(self._last_names):
            self._last_positions = list(pt.positions)
            self._last_stamp = self.get_clock().now()

    def _tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self._last_names
        js.position = self._last_positions
        self._pub.publish(js)


def main():  # pragma: no cover
    rclpy.init()
    node = JointStateShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
