#ifndef HALO_RADAR_HALO_RADAR_H
#define HALO_RADAR_HALO_RADAR_H

#include <ifaddrs.h>
#include <netinet/in.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <map>
#include <chrono>
#include <array>
#include <cstdint>

#include "halo_radar_structures.h"

namespace simrad_halo_radar
{
    
bool validInterface(ifaddrs const *i);

std::vector<uint32_t> getLocalAddresses();

std::string ipAddressToString(uint32_t a);
uint32_t ipAddressFromString(const std::string &a);

struct AddressSet
{
    std::string label;
    std::string serial_number;
    IPAddress data;
    IPAddress send;
    IPAddress report;
    uint32_t interface;
    
    std::string str() const;
};

std::vector <AddressSet> scan();
std::vector <AddressSet> scan(const std::vector<uint32_t> &addresses);

struct Scanline
{
    RawScanline raw;
    std::array<uint8_t, 24> raw_header;
    float angle; // degrees clockwise relative to fwd
    float range; // meters
    bool heading_valid = false;
    bool heading_is_true = false;
    uint16_t ego_heading_raw = 0;
    float ego_heading_degrees = 0.0F;
    std::vector<uint8_t> intensities;
    uint64_t revolution = 0;
};

struct Sector
{
    enum class MessageType : uint8_t
    {
        DATA = 0,
        DATA_MISSING = 1,
        MALFORMED_PACKET = 2,
        ANGLE_DISCONTINUITY = 3
    };

    MessageType message_type = MessageType::DATA;
    MessageType preceding_continuity = MessageType::DATA;
    int64_t arrival_time_ns = 0;
    int64_t previous_arrival_time_ns = 0;
    int64_t arrival_gap_ns = 0;
    uint64_t received_packet_sequence = 0;
    std::string source_address;
    uint16_t source_port = 0;
    uint32_t datagram_size = 0;
    uint32_t socket_message_flags = 0;
    bool datagram_truncated = false;
    bool has_kernel_drop_count = false;
    uint32_t kernel_drop_count = 0;
    uint32_t kernel_drop_count_delta = 0;
    std::array<uint8_t, 8> raw_packet_header{};
    uint8_t declared_scanline_count = 0;
    uint16_t declared_scanline_size = 0;
    bool has_angle_bounds = false;
    uint16_t min_raw_angle = 0;
    uint16_t max_raw_angle = 0;
    uint16_t first_raw_angle = 0;
    uint16_t last_raw_angle = 0;
    // For a synthetic gap this identifies the next angle actually observed;
    // first_raw_angle identifies the first absent angle.
    uint16_t observed_first_raw_angle = 0;
    uint16_t expected_first_raw_angle = 0;
    uint16_t expected_raw_angle_step = 2;
    bool has_valid_heading = false;
    bool heading_consistent = false;
    bool heading_is_true = false;
    uint16_t ego_heading_raw = 0;
    float ego_heading_degrees = 0.0F;
    uint64_t revolution_start = 0;
    uint64_t revolution_end = 0;
    std::vector<uint16_t> missing_raw_angles;
    uint32_t internal_missing_spoke_count = 0;
    std::vector<Scanline> scanlines;
};

struct DatagramMetadata
{
    int64_t arrival_time_ns = 0;
    std::string source_address;
    uint16_t source_port = 0;
    uint32_t socket_message_flags = 0;
    bool datagram_truncated = false;
    bool has_kernel_drop_count = false;
    uint32_t kernel_drop_count = 0;
};

// Stateful decoder shared by the live UDP receiver and deterministic tests.
// One input datagram produces one received sector, optionally preceded by a
// synthetic continuity sector describing missing or ambiguous angles.
class RadarSectorDecoder
{
public:
    std::vector<Sector> decode(
        const uint8_t *data, size_t size, DatagramMetadata const &metadata);
    void reset();

private:
    bool m_havePreviousArrival = false;
    int64_t m_previousArrivalTimeNs = 0;
    bool m_havePreviousSpoke = false;
    uint16_t m_previousRawAngle = 0;
    uint64_t m_revolutionCounter = 0;
    uint64_t m_receivedPacketSequence = 0;
    bool m_haveKernelDropCount = false;
    uint32_t m_previousKernelDropCount = 0;
};

class Radar
{
public:
    Radar(AddressSet const &addresses);
    virtual ~Radar();
    
    void sendCommand(std::string const &key, std::string const &value);
    bool checkHeartbeat();

protected:
    virtual void processData(Sector const &sector)=0;
    virtual void stateUpdated()=0;
    void startThreads();
    void stopThreads();

    std::map <std::string, std::string> m_state;
private:
    void dataThread();
    void reportThread();
    int createListenerSocket(uint32_t interface, uint32_t mcast_address, uint16_t port);
    void sendCommand(const uint8_t data[], int size);
    template<typename T> void sendCommand(const T &data)
    {
        sendCommand(reinterpret_cast<const uint8_t*>(&data),sizeof(T));
    }
    void sendHeartbeat();
    
    AddressSet m_addresses;
    std::thread m_dataThread;
    
    int m_sendSocket = -1;
    sockaddr_in m_sendAddress;
    
    std::thread m_reportThread;
    bool m_exitFlag;
    std::mutex m_exitFlagMutex;
    
    std::chrono::system_clock::time_point m_lastHeartbeat;

    RadarSectorDecoder m_sectorDecoder;
};

class HeadingSender
{
public:
    HeadingSender(uint32_t bindAddress);
    ~HeadingSender();
    void setHeading(double heading);

private:
    void senderThread();

private:
    int m_socket = 0;
    sockaddr_in m_sendAddress;
    double m_heading = 0.0;
    std::mutex m_headingMutex;
    std::thread m_senderThread;
    bool m_exitFlag = false;
    std::mutex m_exitFlagMutex;

    uint16_t m_counter = 0;

    std::chrono::system_clock::time_point m_lastHeadingSent;
    std::chrono::milliseconds m_headingSendInterval = std::chrono::milliseconds(100);
    std::chrono::system_clock::time_point m_lastMysterySent;
    std::chrono::milliseconds m_mysterySendInterval = std::chrono::milliseconds(250);

    HaloHeadingPacket m_headingPacket = {
        {'N', 'K', 'O', 'E'},  // marker
        {0, 1, 0x90, 0x02},    // u00 bytes containing '00 01 90 02'
        0,                     // counter
        {0, 0, 0x10, 0, 0, 0x14, 0, 0, 4, 0, 0, 0, 0, 0, 5, 0x3C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20},  // u01
        {0x12, 0xf1},                                                                                // u02
        {0x01, 0x00},                                                                                // u03
        0,                                                                                           // epoch
        2,                                                                                           // u04
        0,                                                                                           // u05a, likely position
        0,                                                                                           // u05b, likely position
        {0xff},                                                                                      // u06
        0,                                                                                           // heading
        {0xff, 0x7f, 0x79, 0xf8, 0xfc}                                                               // u07
    };

    HaloMysteryPacket m_mysteryPacket  = {
        {'N', 'K', 'O', 'E'},  // marker
        {0, 1, 0x90, 0x02},    // u00 bytes containing '00 01 90 02'
        0,                     // counter
        {0, 0, 0x10, 0, 0, 0x14, 0, 0, 4, 0, 0, 0, 0, 0, 5, 0x3C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20},  // u01
        {0x02, 0xf8},                                                                                // u02
        {0x01, 0x00},                                                                                // u03
        0,                                                                                           // epoch
        2,                                                                                           // u04
        0,                                                                                           // u05a, likely position
        0,                                                                                           // u05b, likely position
        {0xff},                                                                                      // u06
        {0xfc},                                                                                      // u07
        0,                                                                                           // mystery1
        0,                                                                                           // mystery2
        {0xff, 0xff}                                                                                 // u08
    };
};

} // namespace simrad_halo_radar

#endif
