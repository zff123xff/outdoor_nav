# Public command. Fixed IDs: 140 point, 141 patrol, 142 area, 143 inspection, 145 multi-point.
uint16 task_id
# Per-mission sequence number reported as TaskStatus.task_id (0..255).
uint8 mission_seq_id
bool cancel
string[] targets
# Repeated [latitude, longitude, altitude] GPS triplets (degrees, degrees, metres).
float32[] waypoints
# [latitude, longitude, altitude], required for area search only.
float32[] center
float64 length_m
float64 width_m
# ENU degrees counterclockwise from east.
float64 yaw_deg
# Reserved gait speed; currently does not affect task execution.
int16 gait_speed
# Reserved air route number; currently does not affect task execution.
int16 air_route
