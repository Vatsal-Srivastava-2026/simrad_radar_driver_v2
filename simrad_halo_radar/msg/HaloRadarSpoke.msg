# Raw HALO spoke data. Multi-byte numeric fields are decoded from the
# little-endian wire representation; raw_header preserves the exact bytes.

uint16 RAW_ANGLE_MODULUS=4096

uint8[24] raw_header
uint8 header_length
uint8 status
uint16 scan_number
uint16 marker
uint16 large_range
uint16 raw_angle
# Untouched wire word. Bits 0..11 contain heading, bit 14 marks true north;
# any other high bit makes the heading invalid.
uint16 raw_heading
bool heading_valid
bool heading_is_true
uint16 ego_heading_raw
float32 ego_heading_degrees
uint16 small_range
uint16 rotation
uint32 unknown_02
uint32 unknown_03
uint8[512] raw_samples

# Driver-generated revolution containing this spoke. The first observed
# revolution is zero and the counter advances on a forward 4095 -> 0 wrap.
uint64 revolution
