# Simulation observation / marker request. No device or weapon control.
string TARGET_FOUND="target found"
string REQUEST_SHOOT="request shoot"
std_msgs/Header header
sensor_msgs/NavSatFix position
string target_type
uint64 target_id
string command
