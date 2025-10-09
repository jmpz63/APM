# Bridge: JSON → ROS 2 Translator [PLANNED]

Purpose: Parse fabrication JSON and produce robot-friendly work instructions.

Inputs
- JSON array (file or topic)

Outputs
- ROS 2 messages (to be defined): e.g., PanelComponents.msg, WorkInstruction.msg
- Optional services: LoadPanel, StartJob, HaltJob

SLA
- Validate schema; reject malformed inputs with clear errors
- Deterministic ordering and timing model

Implementation
- Python node with rclpy (proto), then C++ if needed for perf
- Unit tests using submodule samples and synthetic_generator output
