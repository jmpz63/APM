# Moveo Trajectory Bridge (Prototype)

ROS 2 node translating `trajectory_msgs/JointTrajectory` into Klipper G-code streamed via Moonraker WebSocket.

## Features (Prototype Stage)
* Subscribes to configurable trajectory topic
* Computes feedrate per segment using max joint delta / dt (converted to degrees if input in radians)
* Streams G1 lines over Moonraker JSON-RPC websocket
* Parameterized axis mapping and buffer timing knobs

## TODO / Roadmap
1. Buffer lookahead and pruning of already-sent segments
2. Preemption handling (new trajectory arrival flush)
3. Connection loss recovery with position re-sync
4. Optionally batch multiple G-code lines in a single JSON-RPC call
5. Velocity smoothing / F low-pass filtering
6. Joint limit validation & safety checks
7. Unit tests with synthetic trajectories
8. Add action server interface to manage lifecycle

## Launch
```bash
source /opt/ros/humble/setup.bash
cd ~/moveo_bridge_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch moveo_trajectory_bridge trajectory_bridge.launch.py
```

## Configuration Parameters
| Name | Description | Default |
|------|-------------|---------|
| trajectory_topic | JointTrajectory topic | /joint_trajectory |
| klipper_ws_url | Moonraker websocket URL | ws://localhost:7125/websocket |
| axis_map | G-code axis letters for joints | [A,B,C,D,E,U] |
| position_unit | 'radians' or 'degrees' input | radians |
| buffer_lead_time | (Reserved) future buffer target | 0.5 |
| min_flush_batch | Minimum lines per flush | 1 |
| max_flush_batch | Maximum lines per flush | 20 |
| flush_period | Seconds between flush checks | 0.05 |

## Notes
Prototype does not yet remove sent segments; currently re-sends from start subset each cycle (harmless for smoke tests but inefficient). Will be replaced by a ring buffer with head index.
