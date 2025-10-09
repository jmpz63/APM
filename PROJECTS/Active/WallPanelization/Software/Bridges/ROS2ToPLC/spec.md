# Bridge: ROS 2 → PLC / Fieldbus [PLANNED]

Purpose: Coordinate workholding, saw/feeder, safety interlocks via PLC or gateway.

Inputs
- ROS 2 job state / interlocks

Outputs
- Profinet/EtherNet-IP/EtherCAT IO mapping, safety signals

SLA
- Interlock latencies < 50 ms where required

Implementation
- PLC program (ladder/st/lib) + ROS 2 gateway; define tag map and states
