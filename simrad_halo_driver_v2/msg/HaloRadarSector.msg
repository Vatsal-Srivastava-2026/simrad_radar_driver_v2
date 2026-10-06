# Loss-aware raw representation of one received HALO UDP sector, or a
# synthetic message describing an angular gap immediately before a sector.

uint8 DATA=0
uint8 DATA_MISSING=1
uint8 MALFORMED_PACKET=2
uint8 ANGLE_DISCONTINUITY=3

std_msgs/Header header
uint8 message_type
string radar_id
string radar_model
string radar_serial_number

# For a received DATA/MALFORMED_PACKET sector, describes continuity from the
# preceding received sector. It is DATA when continuous, DATA_MISSING when
# missing_raw_angles is an exact forward gap, or ANGLE_DISCONTINUITY when the
# transition is ambiguous (duplicate or off-lattice). UDP reordering cannot be
# distinguished from loss and is treated as forward loss by driver policy.
uint8 preceding_continuity

# Monotonic local counter for received data UDP datagrams. A synthetic gap
# message carries the sequence number of the following received datagram.
uint64 received_packet_sequence

# Arrival time of the preceding UDP datagram and the measured gap between it
# and header.stamp. Zero for the first received datagram.
builtin_interfaces/Time previous_packet_stamp
builtin_interfaces/Duration arrival_gap

string source_address
uint16 source_port
uint32 datagram_size
uint32 socket_message_flags
bool datagram_truncated

# Linux SO_RXQ_OVFL metadata. The cumulative counter is scoped to this data
# socket; the delta reports new kernel receive-queue drops since the preceding
# datagram. has_kernel_drop_count is false when the platform supplied no value.
bool has_kernel_drop_count
uint32 kernel_drop_count
uint32 kernel_drop_count_delta

# Exact 8-byte sector header and its decoded count/size fields.
uint8[8] raw_packet_header
uint8 declared_scanline_count
uint16 declared_scanline_size
uint16 parsed_scanline_count

# Numeric extrema and ordered endpoints. first/last must be used for stream
# continuity because min/max are ambiguous across the 4095 -> 0 wrap.
bool has_angle_bounds
uint16 min_raw_angle
uint16 max_raw_angle
uint16 first_raw_angle
uint16 last_raw_angle
# For synthetic continuity messages, the first raw angle actually present in
# the following datagram. This differs from first_raw_angle, which is the first
# absent angle represented by DATA_MISSING.
uint16 observed_first_raw_angle
uint16 expected_first_raw_angle
uint16 expected_raw_angle_step

# Sector heading summary selected from the first spoke carrying a valid
# heading. heading_consistent is true only when all valid spoke headings in
# this sector have the same value and true/magnetic reference.
bool has_valid_heading
bool heading_consistent
bool heading_is_true
uint16 ego_heading_raw
float32 ego_heading_degrees

uint64 revolution_start
uint64 revolution_end

# For DATA_MISSING this is the gap represented by the synthetic message. The
# same list is repeated in the following received sector so that a recorder
# dropping only the synthetic message does not erase the evidence. It is empty
# for ANGLE_DISCONTINUITY because the absent angles cannot be known safely.
uint16[] missing_raw_angles
HaloRadarSpoke[] spokes
