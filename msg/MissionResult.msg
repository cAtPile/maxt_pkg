string mission_id
string status
string[] target_ids
# Aligned with target_ids. A recorded detection stays found for this mission.
bool[] found
string position_target_id
# The first valid position transformed to the configured result frame.
bool position_valid
geometry_msgs/PointStamped position
# Current observation of the selected QR, using the fixed TF child name.
bool report_tf_valid
geometry_msgs/TransformStamped report_tf
bool complete
# First valid WGS84 position of position_target_id in this mission, independent
# of arena position. Header stamp is the QR depth measurement time.
# latitude/longitude: degrees; altitude: metres above the WGS84 ellipsoid.
# frame_id=wgs84 is a geodetic label, NOT a Cartesian TF frame.
bool wgs84_position_valid
sensor_msgs/NavSatFix wgs84_position
string wgs84_status
# Exact input samples selected nearest to the QR measurement time.
time wgs84_fix_stamp
time wgs84_attitude_stamp
# Publication heartbeat, distinct from measurement timestamps in each position.
std_msgs/Header header
uav_qr_mission/QrTargetResult[] targets
bool all_localized
bool survey_finished
string capture_id
string capture_status
bool capture_done
bool capture_success
time capture_stamp
# Readiness of raw RTK + IMU near publication time (not saved-target freshness).
string geolocation_input_status

# Aligned with targets. JPEG from the exposure that confirmed each position.
sensor_msgs/CompressedImage[] target_images
