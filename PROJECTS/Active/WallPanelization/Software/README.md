# Software Structure

Two layers:

1) Design-time Data Pipeline (Authoring)
   - Source of truth for geometry → fabrication JSON
   - Tools: Rhino + Grasshopper GhPython (when available)
   - Alt: Synthetic data generator (no Rhino) for early ROS integration
   - Lives in: APM submodule `repo/` (WallPanelization repo) and references here

2) Runtime Controls & Networking (Execution)
   - ROS 2 nodes that consume fabrication JSON and control hardware
   - Interfaces to PLC/fieldbus/robot controllers
   - Safety and sequencing logic
   - Lives under Components/ControlsAndNetworking (requirements, interfaces) with code in appropriate repos

This separation lets the ROS team start now with synthetic data while the GH pipeline is being developed.
