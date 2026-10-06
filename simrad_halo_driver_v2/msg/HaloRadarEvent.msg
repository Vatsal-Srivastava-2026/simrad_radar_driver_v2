# Structured driver event intended for both live diagnostics and rosbag.

uint8 DATA_MISSING=1
uint8 ANGLE_DISCONTINUITY=2
uint8 MALFORMED_PACKET=3
uint8 KERNEL_DROP=4
uint8 ARRIVAL_GAP=5
uint8 SPOKE_STATUS=6
uint8 REVOLUTION_COMPLETED=7

std_msgs/Header header
uint8 event_type
string radar_id
string radar_model
string radar_serial_number
uint64 received_packet_sequence
uint64 revolution_start
uint64 revolution_end
builtin_interfaces/Duration arrival_gap
uint16 previous_raw_angle
uint16 expected_raw_angle
uint16 observed_raw_angle
uint16[] affected_raw_angles
uint32 kernel_drop_count
uint32 kernel_drop_count_delta
uint8[] spoke_statuses
uint16[] spoke_status_counts
string details
