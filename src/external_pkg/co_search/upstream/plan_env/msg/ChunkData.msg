int32 from_drone_id
int32 to_drone_id
int32 chunk_drone_id

uint32[] voxel_adrs
# Legacy byte-per-voxel field. Empty when occupancy_encoding=1.
uint8[] voxel_occ
uint32 idx

# Compact occupancy payload. Bits are ordered by voxel_adrs index, LSB first.
# occupancy_encoding: 0 = voxel_occ, 1 = voxel_occ_bits.
uint32 voxel_count
uint8 occupancy_encoding
uint8[] voxel_occ_bits

# Simulate swarm communication
uint32 latest_idx
float64 pos_x
float64 pos_y
float64 pos_z