int32 drone_id

# Fixed exploration departure point, used for final return priority.
float32[] start_pos

# Global HGrid identifiers (must span full coarse+fine grid count, often >> 127).
# int8 truncation in the prior definition caused pervasive corrupt ids across the swarm DDS path.
int32[] grid_ids

# Sender-side HGrid global cell count (= local globalGridCount()). If nonzero and !=
# receiver's count, receivers ignore grid_ids (parameter/map topology mismatch).
# Zero = legacy/partial publisher: receivers fall back to sanitize-only on grid_ids.
int32 hgrid_global_cell_count

float64 recent_attempt_time
float64 stamp

# Consensus region assignment: assigned_regions[drone_id-1] is the region allocated
# to that drone by the sender's SNN. All drones arbitrate these proposals to reach
# a consistent global region assignment without a fixed leader.
int32[] assigned_regions
uint64 assignment_epoch
int32 assignment_sender
uint8[] assignment_online_mask

# only used for simulation
float32[] pos
float32[] vel
float32 yaw
int32 cur_state

# Returning flight flag: set when this drone has been released for the final
# return (including the FINISH flight leg). Used by peers to exclude "still
# physically airborne, heading home" drones from the online count so the next
# threshold does NOT trigger early.
bool returning

# Peer coverage (explored) box in the shared ENU frame, piggybacked on this fast
# low-volume topic so anti-clustering does NOT depend on the throttled map-chunk
# channel. cov_valid != 0 => cov_box_min/max are the sender's all_min_/all_max_.
# Receivers use their own receipt time for freshness, so sender/receiver clocks
# need not be synchronized.
int32 cov_valid
float64[3] cov_box_min
float64[3] cov_box_max
