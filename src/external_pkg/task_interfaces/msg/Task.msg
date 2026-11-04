string POINT_GOAL=point_goal
string MULTI_POINT=multi_point
string PATROL_LINE=patrol_line
string AREA_SEARCH=area_search
string task_id
string task_type
# true: cancel the matching task_id; other task fields are ignored.
bool cancel
# Class names or numeric model class IDs; empty means all classes.
string[] targets
sensor_msgs/NavSatFix[] waypoints
task_interfaces/SearchArea area
