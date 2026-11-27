# One registered ground target; position is WGS84 degrees / ellipsoid metres.
string content
bool is_drop_target
bool found
bool position_valid
sensor_msgs/NavSatFix position
string status
time fix_stamp
time attitude_stamp
# First confirmed position is revision 1. Receivers deduplicate by mission/content/revision.
uint32 revision
