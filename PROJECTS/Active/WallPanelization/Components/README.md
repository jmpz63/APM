# WallPanelization Components Catalog

Purpose: Enumerate and decompose all electro-mechanical subsystems required for the field-deployable wall panelization line.

Initial list (expand as we go):
- Robotic arm(s) on linear rails (axis count, reach, payload, controller IO)
- Workholding platform (vacuum/actuated clamps, indexing, datum features)
- Automatic saw with auto feeder (see reference image), infeed/outfeed, safety interlocks
- Material handling (rollers, conveyors, stackers, buffer zones)
- Fastening tool(s) (nailers/screwdrivers), magazine capacity, feed lines
- Sheathing applicator (placement, registration)
- Measurement/inspection (LIDAR/vision, laser lines, encoders)
- Safety systems (E-stops, guarding, light curtains)
- Controls (PLC/IPC, ROS 2 nodes, fieldbus, network)
- Pneumatics and hydraulics (compressor, valves, actuators)

For each component, create a folder with:
- spec.md — key requirements, environment, duty cycle
- interfaces.md — electrical, mechanical, control signals
- power.md — nameplate power, typical power, duty cycle, inrush
- risks.md — failure modes, mitigations
- bom.csv — vendor, part no., qty, cost
