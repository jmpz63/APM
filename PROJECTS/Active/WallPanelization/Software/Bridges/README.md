# Software Bridges (Catalog)

Purpose: Track interfaces between tools, file formats, and controllers.

Status keys: [EXISTS], [PLANNED], [WIP]

- Geometry → JSON Exporter [EXISTS]
  - Location: submodule repo/ (GhPython exporter) and Software/synthetic_generator.py
  - Contract: docs/JSON_CONTRACT.md, schema/fabrication.schema.json
- JSON → ROS 2 Translator [PLANNED]
  - Load JSON and produce robot work instructions/messages
  - Target: ros2 msgs/services (to be defined), timing model
- ROS 2 → Robot Controller (Arm on Rails) [PLANNED]
  - Interface: vendor driver or ROS-I; motion planning, tool IO
- ROS 2 → PLC/Fieldbus (Workholding/Saw/Feeder) [PLANNED]
  - Interface: Profinet/EtherNet-IP/EtherCAT gateway; state machine & interlocks
- ROS 2 ↔ Vision/Inspection [PLANNED]
  - Interface: camera/LiDAR driver; calibration and pose streams
- ROS 2 → Logger/Telemetry [PLANNED]
  - Interface: timeseries DB / files; batch record per panel
- JSON Validator & CI [EXISTS]
  - Location: submodule repo/tools and CI workflow
- Power Data → Reports [EXISTS]
  - Location: Power/compute_power.py, Power/*.md

Add new bridges as folders here with spec.md (purpose, inputs/outputs, SLAs), proto.md (message shapes), and impl.md (repo links, tasks).
