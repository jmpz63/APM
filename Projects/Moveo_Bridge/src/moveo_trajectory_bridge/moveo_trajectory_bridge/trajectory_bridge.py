import json
import math
import time
from dataclasses import dataclass
from typing import List, Optional
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

try:  # pragma: no cover
    import websockets  # type: ignore
except ImportError:  # pragma: no cover
    websockets = None  # type: ignore


@dataclass
class Segment:
    positions: List[float]
    time_from_start: float


class MoveoTrajectoryBridge(Node):
    """Baseline stable JointTrajectory -> Klipper G-code bridge.

    Included features:
      - Debounce and replacement gating
      - Duplicate trajectory hash suppression
      - Dry-run batching & logging
      - Feedrate from max joint delta / dt (converted to deg/min if radians)
    Advanced features intentionally removed for stability.
    """

    def __init__(self):
        super().__init__('moveo_trajectory_bridge')

        # Parameters
        self.declare_parameter('trajectory_topic', '/joint_trajectory')
        self.declare_parameter('klipper_ws_url', 'ws://localhost:7125/websocket')
        self.declare_parameter('axis_map', ['A','B','C','D','E','U'])
        self.declare_parameter('position_unit', 'radians')  # incoming unit
        self.declare_parameter('min_flush_batch', 1)
        self.declare_parameter('max_flush_batch', 20)
        self.declare_parameter('flush_period', 0.05)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('log_sample_lines', 3)
        self.declare_parameter('require_stream_macro', True)
        self.declare_parameter('max_live_segments', -1)
        self.declare_parameter('debounce_ms', 100)
        self.declare_parameter('replace_threshold_pct', 80)
        # If true, once a trajectory is fully consumed (all segments sent), allow an identical
        # trajectory hash to be accepted again (enables replay of same motion).
        self.declare_parameter('clear_signature_when_consumed', True)
        self.declare_parameter('joint_velocity_caps', [])  # optional per-joint caps (deg/s or rad/s matching input unit)
        # Payload / dynamic scaling
        self.declare_parameter('payload_mode', 'normal')  # normal | heavy
        self.declare_parameter('joint2_heavy_scale_vel', 0.7)  # scale factor on J2 velocity when heavy
        self.declare_parameter('joint2_heavy_scale_accel', 0.6)  # scale factor on feed proxy when heavy
        self.declare_parameter('joint2_inertia_proxy', 0.05)  # rough kg*m^2 placeholder
        self.declare_parameter('joint2_effort_margin', 0.8)  # warn if (effort_est/allowed) > margin
        # Joint limit enforcement (list flattened: [min1,max1,min2,max2,...])
        # Provide a typed double[] default (empty list can be mis-typed by rclpy)
        self.declare_parameter('joint_position_limits', [0.0, 0.0])  # placeholder even-length; override in launch
        self.declare_parameter('limit_mode', 'clamp')  # clamp | reject
        # Optional manual stepper mode (single-joint hardware bring-up)
        self.declare_parameter('manual_stepper_mode', False)
        self.declare_parameter('manual_stepper_joint_index', 0)
        self.declare_parameter('manual_stepper_name', 'joint1')
        self.declare_parameter('manual_stepper_units', 'deg')  # deg | raw (no conversion)
        self.declare_parameter('manual_stepper_default_speed', 45.0)  # units/sec (deg/s if deg)
        self.declare_parameter('manual_stepper_speed_cap', 90.0)  # cap units/sec
        # Transport selection (temporary pragmatic implementation)
        self.declare_parameter('use_http_transport', True)  # True: use Moonraker HTTP POST, False: (future) websocket
        self.declare_parameter('moonraker_base_url', 'http://localhost:7125')
        self.declare_parameter('moonraker_api_key', '')
        self.declare_parameter('log_http_success_body', False)

        gp = self.get_parameter
        self._traj_topic = gp('trajectory_topic').value
        self._ws_url = gp('klipper_ws_url').value
        self._axis_map = gp('axis_map').value
        self._unit = gp('position_unit').value
        self._min_batch = int(gp('min_flush_batch').value)
        self._max_batch = int(gp('max_flush_batch').value)
        self._flush_period = float(gp('flush_period').value)
        self._dry_run = bool(gp('dry_run').value)
        self._log_sample = int(gp('log_sample_lines').value)
        self._require_stream_macro = bool(gp('require_stream_macro').value)
        self._max_live_segments = int(gp('max_live_segments').value)
        self._debounce_ms = int(gp('debounce_ms').value)
        self._replace_threshold_pct = int(gp('replace_threshold_pct').value)
        self._clear_sig_consumed = bool(gp('clear_signature_when_consumed').value)
        self._joint_velocity_caps = gp('joint_velocity_caps').value
        self._payload_mode = gp('payload_mode').value
        self._j2_heavy_scale_vel = float(gp('joint2_heavy_scale_vel').value)
        self._j2_heavy_scale_accel = float(gp('joint2_heavy_scale_accel').value)
        self._j2_inertia = float(gp('joint2_inertia_proxy').value)
        self._j2_effort_margin = float(gp('joint2_effort_margin').value)
        self._use_http = bool(gp('use_http_transport').value)
        self._base_url = gp('moonraker_base_url').value.rstrip('/')
        self._http_endpoint = f"{self._base_url}/printer/gcode/script"
        self._api_key = gp('moonraker_api_key').value
        self._log_http_body = bool(gp('log_http_success_body').value)
        # Limits
        _flat_limits = gp('joint_position_limits').value
        # Accept string form (e.g. passed via CLI as "[-1.0,1.0,...]")
        if isinstance(_flat_limits, str):
            try:
                parsed = json.loads(_flat_limits)
                if isinstance(parsed, list):
                    _flat_limits = parsed
            except Exception:
                self.get_logger().warn('Failed to parse joint_position_limits string; ignoring')
        self._limit_mode = gp('limit_mode').value
        self._joint_limits = []  # type: List[Optional[tuple]]
        if _flat_limits:
            if len(_flat_limits) % 2 != 0:
                self.get_logger().warn('joint_position_limits length not even; ignoring')
            else:
                pair_count = len(_flat_limits)//2
                if pair_count != len(self._axis_map):
                    self.get_logger().warn('joint_position_limits pair count != axis count; ignoring')
                else:
                    for i in range(pair_count):
                        mn = float(_flat_limits[2*i])
                        mx = float(_flat_limits[2*i+1])
                        if mn > mx:
                            mn, mx = mx, mn
                        self._joint_limits.append((mn, mx))
        if not self._joint_limits:
            # fill with Nones for simpler downstream logic
            self._joint_limits = [None]*len(self._axis_map)
        # Manual stepper
        self._manual_stepper_mode = bool(gp('manual_stepper_mode').value)
        self._ms_joint_index = int(gp('manual_stepper_joint_index').value)
        self._ms_name = gp('manual_stepper_name').value
        self._ms_units = gp('manual_stepper_units').value
        self._ms_default_speed = float(gp('manual_stepper_default_speed').value)
        self._ms_speed_cap = float(gp('manual_stepper_speed_cap').value)
        if self._manual_stepper_mode and (self._ms_joint_index < 0 or self._ms_joint_index >= len(self._axis_map)):
            self.get_logger().warn('manual_stepper_joint_index out of range; disabling manual stepper mode')
            self._manual_stepper_mode = False
        if self._joint_velocity_caps and len(self._joint_velocity_caps) != len(self._axis_map):
            self.get_logger().warn('joint_velocity_caps length mismatch; ignoring caps')
            self._joint_velocity_caps = []

        # State
        self._segments = []
        self._head_index = 0
        self._current_traj_id = None
        self._last_traj_signature = None
        self._last_accept_time = 0.0
        self._ws = None
        self._last_send_time = time.time()

        # Subscription
        self.subscription = self.create_subscription(
            JointTrajectory,
            self._traj_topic,
            self._trajectory_cb,
            10
        )
        self.get_logger().info(f"Subscribed to trajectory topic: {self._traj_topic}")

        # Shadow joint states publisher (command echo). Provides minimal /joint_states for planning continuity.
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._js_timer = self.create_timer(0.1, self._publish_joint_states)
        self._last_positions: List[float] = [0.0]*len(self._axis_map)

        # Flush handled via ROS timer (simpler & reliable inside rclpy spin)
        self._flush_timer = self.create_timer(self._flush_period, self._flush_tick)

    def _ensure_connection(self):  # pragma: no cover
        # Placeholder: in future convert to non-blocking connection management.
        pass

    def _trajectory_cb(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Received empty JointTrajectory')
            return
        now_ms = time.time() * 1000.0
        if (now_ms - self._last_accept_time) < self._debounce_ms:
            self.get_logger().debug('Debounced trajectory (too soon)')
            return

        sig_parts: List[str] = []
        new_segments: List[Segment] = []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            pos_list = list(pt.positions)
            # Enforce limits as configured
            violate = False
            for j, lim in enumerate(self._joint_limits):
                if lim is None or j >= len(pos_list):
                    continue
                mn, mx = lim
                v = pos_list[j]
                if v < mn or v > mx:
                    if self._limit_mode == 'reject':
                        violate = True
                        break
                    # clamp
                    clamped = max(mn, min(mx, v))
                    if clamped != v:
                        self.get_logger().debug(f"Clamped joint{j+1} {v:.5f}->{clamped:.5f}")
                        pos_list[j] = clamped
            if violate:
                self.get_logger().warn('Trajectory rejected (joint limit violation)')
                return
            new_segments.append(Segment(pos_list, t))
            sig_parts.append(f"{t:.6f}:")
            for v in pos_list:
                sig_parts.append(f"{v:.5f},")
        new_sig = hash(''.join(sig_parts))
        if self._last_traj_signature is not None and new_sig == self._last_traj_signature:
            self.get_logger().info('Duplicate trajectory ignored (hash match with active or un-cleared previous)')
            return

        if self._segments:
            total_old = len(self._segments)
            threshold_old = int(self._replace_threshold_pct/100.0 * total_old) if total_old else 0
            if total_old > 0 and self._head_index < threshold_old:
                self.get_logger().info(
                    f"New trajectory ignored: sent {self._head_index}/{total_old} (<{self._replace_threshold_pct}%).")
                return

        self._current_traj_id = int(time.time() * 1000)
        self._segments = new_segments
        self._head_index = 0
        self._last_traj_signature = new_sig
        self._last_accept_time = now_ms
        self.get_logger().info(
            f"Loaded trajectory id={self._current_traj_id} segments={len(self._segments)} (accepted)"
        )
        # Update shadow positions to first point if present
        if self._segments:
            first = self._segments[0]
            for i, v in enumerate(first.positions):
                if i < len(self._last_positions):
                    self._last_positions[i] = v

    def _format_position(self, value: float) -> float:
        if self._unit == 'radians':
            return math.degrees(value)
        return value

    def _build_gcode(self, positions: List[float], feed_rate: float) -> str:
        parts = ['G1']
        for axis, pos in zip(self._axis_map, positions):
            parts.append(f"{axis}{self._format_position(pos):.5f}")
        parts.append(f"F{feed_rate:.2f}")
        return ' '.join(parts)

    def _compute_feed(self, prev: Segment, cur: Segment) -> float:
        dt = cur.time_from_start - prev.time_from_start
        if dt <= 0:
            return 0.0
        deltas = [abs(c - p) for p, c in zip(prev.positions, cur.positions)]
        max_delta = max(deltas) if deltas else 0.0
        if self._unit == 'radians':
            max_delta = math.degrees(max_delta)
        v = max_delta / dt
        feed = v * 60.0  # deg/min if unit was radians (converted above) else unit/min
        # Optional scaling if per-joint caps provided.
        if self._joint_velocity_caps and dt > 0:
            # Compute velocity per joint in input units; convert to deg if unit==radians for comparison.
            scale = 1.0
            for idx, (pval, cval) in enumerate(zip(prev.positions, cur.positions)):
                vel = abs(cval - pval)/dt
                cap = self._joint_velocity_caps[idx]
                if cap in (None, 0):
                    continue
                v_check = math.degrees(vel) if self._unit == 'radians' else vel
                if v_check > cap:
                    scale = min(scale, cap / v_check)
            if scale < 1.0:
                feed *= scale
        # Heavy payload scaling (only affects J2 related dynamics heuristically)
        if self._payload_mode == 'heavy':
            # Further reduce feed if J2 is the limiting joint
            # We approximate J2 delta velocity vs others
            if len(prev.positions) > 1:
                j2_delta = abs(cur.positions[1] - prev.positions[1]) / dt
                if self._unit == 'radians':
                    j2_delta = math.degrees(j2_delta)
                # If J2 dominates the max delta we scaled on, apply heavy scale
                if j2_delta * 60.0 > 0.5 * feed:  # heuristic dominance check
                    feed *= self._j2_heavy_scale_vel
        return feed

    def _stream_active(self) -> bool:
        # Placeholder: stream always considered active. Hook for future macro.
        if self._dry_run or not self._require_stream_macro:
            return True
        return True

    def _build_batch(self) -> List[str]:
        if len(self._segments) - self._head_index <= 1:
            return []
        batch: List[str] = []
        start_i = max(1, self._head_index + 1)
        if self._manual_stepper_mode:
            # Only emit commands for selected joint; convert to requested units.
            j = self._ms_joint_index
            prev = self._segments[start_i - 1]
            for i in range(start_i, len(self._segments)):
                cur = self._segments[i]
                dt = cur.time_from_start - prev.time_from_start
                if dt <= 0:
                    prev = cur
                    continue
                if j >= len(cur.positions):
                    break
                delta = cur.positions[j] - prev.positions[j]
                val = delta
                if self._unit == 'radians' and self._ms_units == 'deg':
                    val = math.degrees(val)
                # speed estimate (units/sec)
                speed = abs(val) / dt if dt > 0 else self._ms_default_speed
                if speed <= 0:
                    prev = cur
                    continue
                speed = max(min(speed, self._ms_speed_cap), 1e-3)
                # If speed tiny but movement non-zero, enforce default
                if speed < 1e-3 and abs(val) > 0:
                    speed = self._ms_default_speed
                # Build command. NOTE: Assumes Klipper manual_stepper units already calibrated to provided units.
                line = f"MANUAL_STEPPER STEPPER={self._ms_name} MOVE={val:.5f} SPEED={speed:.5f}"
                batch.append(line)
                prev = cur
                if len(batch) >= self._max_batch:
                    break
            return batch
        for i in range(start_i, len(self._segments)):
            prev = self._segments[i - 1]
            cur = self._segments[i]
            feed = self._compute_feed(prev, cur)
            batch.append(self._build_gcode(cur.positions, feed))
            if len(batch) >= self._max_batch:
                break
        return batch

    def _flush_tick(self):
        try:
            if not self._segments:
                return
            # Only gate on websocket connection if we are in websocket mode. For HTTP we don't need _ws.
            if (not self._dry_run) and (not self._use_http) and (self._ws is None or getattr(self._ws, 'closed', True)):
                return
            if not self._stream_active():
                return
            batch_cmds = self._build_batch()
            if not batch_cmds:
                return
            # Effort proxy warning (uses last pair prev->cur for J2 only)
            if self._payload_mode in ('heavy', 'normal') and self._head_index+1 < len(self._segments):
                prev = self._segments[self._head_index]
                cur = self._segments[self._head_index+1]
                dt = cur.time_from_start - prev.time_from_start
                if dt > 0 and len(prev.positions) > 1:
                    # Approx angular acceleration for J2 over this segment boundary
                    a = abs(cur.positions[1] - prev.positions[1]) / dt / dt
                    if self._unit == 'radians':
                        a = a  # already rad/s^2
                        inertia = self._j2_inertia
                    else:
                        # convert deg to rad for inertia*accel product
                        a = math.radians(a)
                        inertia = self._j2_inertia
                    effort_est = inertia * a  # simplistic torque proxy (N*m) if inertia in kg*m^2
                    # Allowed proxy: scale with payload mode (heavy reduces allowance)
                    allowed = (self._j2_effort_margin if self._payload_mode == 'normal' else self._j2_effort_margin * self._j2_heavy_scale_accel)
                    if effort_est > allowed:
                        self.get_logger().warn(
                            f"Joint2 effort proxy high: est={effort_est:.3f} > allowed={allowed:.3f} (mode={self._payload_mode})"
                        )
            if not self._dry_run and self._max_live_segments > -1:
                remaining_allowed = self._max_live_segments - self._head_index
                if remaining_allowed <= 0:
                    return
                if len(batch_cmds) > remaining_allowed:
                    batch_cmds = batch_cmds[:remaining_allowed]
            sample = batch_cmds[:self._log_sample]
            if self._dry_run:
                self.get_logger().info(
                    f"[DRY-RUN] batch={len(batch_cmds)} head={self._head_index}->{self._head_index + len(batch_cmds)} sample=" +
                    " | ".join(sample)
                )
            else:  # LIVE path
                sent = 0
                if self._use_http:
                    for line in batch_cmds:
                        body = json.dumps({'script': line}).encode('utf-8')
                        headers = {'Content-Type': 'application/json'}
                        if self._api_key:
                            headers['X-Api-Key'] = self._api_key
                        req = urllib.request.Request(self._http_endpoint, data=body, headers=headers)
                        try:
                            with urllib.request.urlopen(req, timeout=2) as resp:  # noqa: F841
                                if self._log_http_body:
                                    try:
                                        rtxt = resp.read().decode('utf-8')
                                        self.get_logger().debug(f'HTTP ok body: {rtxt[:200]}')
                                    except Exception:  # pragma: no cover
                                        pass
                                sent += 1
                        except urllib.error.HTTPError as e:  # pragma: no cover
                            self.get_logger().error(f'HTTP error {e.code} sending line {sent}: {e.reason}')
                            break
                        except Exception as e:  # pragma: no cover
                            self.get_logger().error(f'HTTP send failure after {sent} lines: {e}')
                            break
                else:
                    if self._ws is None:
                        self.get_logger().warn('Websocket mode not implemented; enable use_http_transport or add websocket connect logic.')
                    # No websocket implementation provided yet (placeholder)
                self.get_logger().info(
                    f"[LIVE] sent={sent} batch head={self._head_index}->{self._head_index + sent} sample=" +
                    " | ".join(sample)
                )
            self._head_index += len(batch_cmds)
            # If we've consumed up to final segment (head at last index), clear signature to allow replay
            if self._clear_sig_consumed and self._segments and self._head_index >= len(self._segments)-1:
                if self._last_traj_signature is not None:
                    self.get_logger().debug('Trajectory fully consumed; clearing signature for possible replay')
                self._last_traj_signature = None
            if self._head_index > 1000:
                keep_tail = self._segments[self._head_index - 1:]
                self._segments = keep_tail
                self._head_index = 1
            self._last_send_time = time.time()
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f'Flush tick error: {e}')

    def _publish_joint_states(self):
        try:
            js = JointState()
            now = self.get_clock().now().to_msg()
            js.header.stamp = now
            # Provide a consistent joint name list (same order as axis_map but using synthetic joint names joint1..)
            js.name = [f'joint{i+1}' for i in range(len(self._axis_map))]
            # Use last commanded positions (zero if never commanded). No velocities/efforts.
            js.position = list(self._last_positions)
            self._js_pub.publish(js)
        except Exception as e:  # pragma: no cover
            self.get_logger().debug(f'JointState publish error: {e}')

    def destroy_node(self):  # pragma: no cover
        return super().destroy_node()


def main():  # pragma: no cover
    rclpy.init()
    node = MoveoTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
