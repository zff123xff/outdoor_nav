MultiGoalPoint[]        multi_goal_points        # array of MultiGoalPoint
bool is_gps_aid                                  # AID of data  False:LOCAL(x,y)  True:GPS(latitude,longitude)
bool is_gps_hgt                                  # HGT of data  False:LOCAL_z     True:GPS_altitude
int8 ctl_mode                                    # 0:vel & yawrate   1:vel & yaw
bool is_patrol                                    # Ping-pong between the first two waypoints
bool cancel_patrol                                # Cancel active ping-pong patrol
uint32 patrol_max_laps                            # A->B->A count; 0 means repeat indefinitely
float64 patrol_hold_time_at_endpoint_s            # Dwell at each reached endpoint (seconds)
