#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "halo_radar.h"
#include "halo_radar_node.hpp"

namespace {
using namespace std::chrono_literals;
using simrad_halo_radar::DatagramMetadata;
using simrad_halo_radar::RadarSectorDecoder;
using simrad_halo_radar::Sector;

constexpr size_t kSectorHeaderSize = 8;
constexpr size_t kSpokeSize = 536;

struct SpokeSpec {
  uint16_t angle = 0;
  uint16_t scan_number = 0;
  uint8_t status = 0x02;
  uint16_t heading = 0x4400;
  uint16_t large_range = 128;
  uint16_t small_range = 400;
  uint8_t first_sample = 0xa3;
};

void writeLe16(uint8_t *data, uint16_t value) {
  data[0] = value & 0xff;
  data[1] = value >> 8;
}

void writeLe32(uint8_t *data, uint32_t value) {
  data[0] = value & 0xff;
  data[1] = (value >> 8) & 0xff;
  data[2] = (value >> 16) & 0xff;
  data[3] = (value >> 24) & 0xff;
}

std::vector<uint8_t> makePacket(const std::vector<SpokeSpec> &spokes,
                                uint16_t declared_size = kSpokeSize,
                                int declared_count = -1) {
  const size_t stride = std::max<size_t>(declared_size, kSpokeSize);
  std::vector<uint8_t> packet(kSectorHeaderSize + spokes.size() * stride, 0);
  packet[0] = 0x01;
  packet[1] = 0x02;
  packet[2] = 0x03;
  packet[3] = 0x04;
  packet[4] = 0x05;
  packet[5] =
      static_cast<uint8_t>(declared_count < 0 ? spokes.size() : declared_count);
  writeLe16(packet.data() + 6, declared_size);

  for (size_t i = 0; i < spokes.size(); ++i) {
    uint8_t *wire = packet.data() + kSectorHeaderSize + i * stride;
    const auto &spoke = spokes[i];
    wire[0] = 24;
    wire[1] = spoke.status;
    writeLe16(wire + 2, spoke.scan_number);
    writeLe16(wire + 4, 0x4400);
    writeLe16(wire + 6, spoke.large_range);
    writeLe16(wire + 8, spoke.angle);
    writeLe16(wire + 10, spoke.heading);
    writeLe16(wire + 12, spoke.small_range);
    writeLe16(wire + 14, spoke.angle);
    writeLe32(wire + 16, 0x80000000U);
    writeLe32(wire + 20, 0xa0000000U);
    for (size_t sample = 0; sample < 512; ++sample)
      wire[24 + sample] =
          static_cast<uint8_t>((sample + spoke.first_sample) & 0xff);
    wire[24] = spoke.first_sample;
  }
  return packet;
}

DatagramMetadata metadata(int64_t timestamp_ns, uint32_t drops = 0) {
  DatagramMetadata result;
  result.arrival_time_ns = timestamp_ns;
  result.source_address = "127.0.0.1";
  result.source_port = 42000;
  result.has_kernel_drop_count = true;
  result.kernel_drop_count = drops;
  return result;
}

uint16_t reserveUdpPort() {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd < 0) {
    ADD_FAILURE() << "could not create UDP socket";
    return 0;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (bind(socket_fd, reinterpret_cast<sockaddr *>(&address),
           sizeof(address)) != 0) {
    ADD_FAILURE() << "could not bind UDP socket";
    close(socket_fd);
    return 0;
  }
  socklen_t length = sizeof(address);
  if (getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address), &length) !=
      0) {
    ADD_FAILURE() << "could not read assigned UDP port";
    close(socket_fd);
    return 0;
  }
  const uint16_t port = ntohs(address.sin_port);
  close(socket_fd);
  return port;
}

simrad_halo_radar::AddressSet makeAddresses(const std::string &label,
                                            const char *data_group,
                                            const char *report_group) {
  simrad_halo_radar::AddressSet addresses{};
  addresses.label = label;
  addresses.serial_number = "SYNTHETIC-HALO24";
  addresses.interface = inet_addr("127.0.0.1");
  addresses.data.address = inet_addr(data_group);
  addresses.data.port = htons(reserveUdpPort());
  addresses.report.address = inet_addr(report_group);
  addresses.report.port = htons(reserveUdpPort());
  addresses.send.address = inet_addr("127.0.0.1");
  addresses.send.port = htons(reserveUdpPort());
  return addresses;
}

bool sendMulticast(const simrad_halo_radar::IPAddress &destination,
                   const std::vector<uint8_t> &packet) {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd < 0)
    return false;
  in_addr loopback{};
  loopback.s_addr = inet_addr("127.0.0.1");
  unsigned char enabled = 1;
  unsigned char ttl = 0;
  bool ok = setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_IF, &loopback,
                       sizeof(loopback)) == 0 &&
            setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_LOOP, &enabled,
                       sizeof(enabled)) == 0 &&
            setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl,
                       sizeof(ttl)) == 0;
  sockaddr_in target{};
  target.sin_family = AF_INET;
  target.sin_addr.s_addr = destination.address;
  target.sin_port = destination.port;
  if (ok) {
    ok = sendto(socket_fd, packet.data(), packet.size(), 0,
                reinterpret_cast<sockaddr *>(&target),
                sizeof(target)) == static_cast<ssize_t>(packet.size());
  }
  close(socket_fd);
  return ok;
}

class CapturingRadar : public simrad_halo_radar::Radar {
public:
  explicit CapturingRadar(const simrad_halo_radar::AddressSet &addresses)
      : Radar(addresses) {
    startThreads();
  }

  ~CapturingRadar() { stopThreads(); }

  bool waitFor(size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout,
                               [&]() { return sectors_.size() >= count; });
  }

  std::vector<Sector> sectors() {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sectors_;
  }

private:
  void processData(const Sector &sector) override {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      sectors_.push_back(sector);
    }
    condition_.notify_all();
  }

  void stateUpdated() override {}

  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<Sector> sectors_;
};

TEST(RadarSectorDecoderTest, DecodesAllWireFieldsAndUnpackedIntensities) {
  RadarSectorDecoder decoder;
  const auto packet =
      makePacket({SpokeSpec{100, 254, 0x02, 0x4400, 128, 400, 0xa3},
                  SpokeSpec{102, 255, 0x12, 0x0200, 4, 512, 0x5e}});
  const auto decoded =
      decoder.decode(packet.data(), packet.size(), metadata(1000000000LL, 7));

  ASSERT_EQ(decoded.size(), 1U);
  const auto &sector = decoded.front();
  EXPECT_EQ(sector.message_type, Sector::MessageType::DATA);
  EXPECT_EQ(sector.received_packet_sequence, 0U);
  EXPECT_EQ(sector.datagram_size, packet.size());
  EXPECT_EQ(sector.declared_scanline_count, 2U);
  EXPECT_EQ(sector.declared_scanline_size, kSpokeSize);
  EXPECT_EQ(sector.first_raw_angle, 100U);
  EXPECT_EQ(sector.last_raw_angle, 102U);
  EXPECT_TRUE(sector.has_valid_heading);
  EXPECT_FALSE(sector.heading_consistent);
  EXPECT_TRUE(sector.heading_is_true);
  EXPECT_EQ(sector.ego_heading_raw, 1024U);
  EXPECT_FLOAT_EQ(sector.ego_heading_degrees, 90.0F);
  EXPECT_EQ(sector.kernel_drop_count, 7U);
  EXPECT_EQ(sector.kernel_drop_count_delta, 7U);

  ASSERT_EQ(sector.scanlines.size(), 2U);
  const auto &first = sector.scanlines[0];
  EXPECT_EQ(first.raw.scan_number, 254U);
  EXPECT_EQ(first.raw.status, 0x02U);
  EXPECT_EQ(first.raw.angle, 100U);
  EXPECT_EQ(first.raw.heading, 0x4400U);
  EXPECT_FLOAT_EQ(first.range, 100.0F);
  ASSERT_EQ(first.intensities.size(), 1024U);
  EXPECT_EQ(first.raw.data[0], 0xa3U);
  EXPECT_EQ(first.intensities[0], 3U);
  EXPECT_EQ(first.intensities[1], 10U);
  EXPECT_EQ(first.raw_header[0], 24U);
  EXPECT_EQ(first.raw_header[1], 0x02U);

  const auto &second = sector.scanlines[1];
  EXPECT_EQ(second.raw.status, 0x12U);
  EXPECT_FLOAT_EQ(second.range, 4.0F);
  EXPECT_TRUE(second.heading_valid);
  EXPECT_FALSE(second.heading_is_true);
  EXPECT_EQ(second.ego_heading_raw, 512U);
}

TEST(RadarSectorDecoderTest, EmitsExactMissingAnglesAcrossRevolutionWrap) {
  RadarSectorDecoder decoder;
  auto first = makePacket({SpokeSpec{4092, 1}, SpokeSpec{4094, 2}});
  auto second = makePacket({SpokeSpec{4, 3}, SpokeSpec{6, 4}});
  ASSERT_EQ(decoder.decode(first.data(), first.size(), metadata(1000)).size(),
            1U);
  const auto decoded =
      decoder.decode(second.data(), second.size(), metadata(2500));

  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::DATA_MISSING);
  EXPECT_EQ(decoded[0].missing_raw_angles, (std::vector<uint16_t>{0, 2}));
  EXPECT_EQ(decoded[0].first_raw_angle, 0U);
  EXPECT_EQ(decoded[0].last_raw_angle, 2U);
  EXPECT_EQ(decoded[0].observed_first_raw_angle, 4U);
  EXPECT_EQ(decoded[0].revolution_start, 1U);
  EXPECT_EQ(decoded[1].message_type, Sector::MessageType::DATA);
  EXPECT_EQ(decoded[1].preceding_continuity, Sector::MessageType::DATA_MISSING);
  EXPECT_EQ(decoded[1].missing_raw_angles, (std::vector<uint16_t>{0, 2}));
  EXPECT_EQ(decoded[1].arrival_gap_ns, 1500);
  EXPECT_EQ(decoded[1].received_packet_sequence, 1U);
  EXPECT_EQ(decoded[1].revolution_start, 1U);
  EXPECT_EQ(decoded[1].revolution_end, 1U);
}

TEST(RadarSectorDecoderTest, HandlesContinuousWrapWithoutMissingData) {
  RadarSectorDecoder decoder;
  auto first = makePacket({SpokeSpec{4092, 1}, SpokeSpec{4094, 2}});
  auto second = makePacket({SpokeSpec{0, 3}, SpokeSpec{2, 4}});
  decoder.decode(first.data(), first.size(), metadata(1000));
  const auto decoded =
      decoder.decode(second.data(), second.size(), metadata(2000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].preceding_continuity, Sector::MessageType::DATA);
  EXPECT_TRUE(decoded[0].missing_raw_angles.empty());
  EXPECT_EQ(decoded[0].revolution_start, 1U);
}

TEST(RadarSectorDecoderTest, FlagsDuplicateAndOffLatticeTransitions) {
  RadarSectorDecoder decoder;
  auto first = makePacket({SpokeSpec{10, 1}, SpokeSpec{12, 2}});
  auto duplicate = makePacket({SpokeSpec{12, 3}, SpokeSpec{14, 4}});
  decoder.decode(first.data(), first.size(), metadata(1000));
  auto decoded =
      decoder.decode(duplicate.data(), duplicate.size(), metadata(2000));
  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::ANGLE_DISCONTINUITY);
  EXPECT_TRUE(decoded[0].missing_raw_angles.empty());
  EXPECT_EQ(decoded[0].expected_first_raw_angle, 14U);
  EXPECT_EQ(decoded[0].observed_first_raw_angle, 12U);

  decoder.reset();
  decoder.decode(first.data(), first.size(), metadata(3000));
  auto odd = makePacket({SpokeSpec{15, 5}});
  decoded = decoder.decode(odd.data(), odd.size(), metadata(4000));
  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::ANGLE_DISCONTINUITY);
}

TEST(RadarSectorDecoderTest, RejectsShortTruncatedAndInvalidSizePacketsSafely) {
  RadarSectorDecoder decoder;
  const uint8_t short_packet[] = {1, 2, 3, 4};
  auto decoded =
      decoder.decode(short_packet, sizeof(short_packet), metadata(1000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  EXPECT_TRUE(decoded[0].scanlines.empty());

  auto truncated =
      makePacket({SpokeSpec{10, 1}, SpokeSpec{12, 2}}, kSpokeSize, 3);
  decoded = decoder.decode(truncated.data(), truncated.size(), metadata(2000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  EXPECT_EQ(decoded[0].scanlines.size(), 2U);

  auto wrong_size = makePacket({SpokeSpec{20, 3}}, 535);
  decoded =
      decoder.decode(wrong_size.data(), wrong_size.size(), metadata(3000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  EXPECT_TRUE(decoded[0].scanlines.empty());

  auto valid = makePacket({SpokeSpec{30, 4}});
  auto truncated_metadata = metadata(4000);
  truncated_metadata.datagram_truncated = true;
  decoded = decoder.decode(valid.data(), valid.size(), truncated_metadata);
  ASSERT_EQ(decoded.size(),
            2U); // continuity event plus received malformed sector
  EXPECT_EQ(decoded.back().message_type, Sector::MessageType::MALFORMED_PACKET);
}

TEST(RadarSectorDecoderTest, AcceptsMaximum120SpokePacketAndTracksDropDelta) {
  RadarSectorDecoder decoder;
  std::vector<SpokeSpec> spokes;
  for (uint16_t i = 0; i < 120; ++i)
    spokes.push_back(SpokeSpec{static_cast<uint16_t>(100 + 2 * i), i});
  const auto packet = makePacket(spokes);
  auto decoded =
      decoder.decode(packet.data(), packet.size(), metadata(1000, 10));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::DATA);
  EXPECT_EQ(decoded[0].scanlines.size(), 120U);
  EXPECT_EQ(decoded[0].kernel_drop_count_delta, 10U);

  auto next = makePacket({SpokeSpec{340, 120}});
  decoded = decoder.decode(next.data(), next.size(), metadata(2000, 13));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].kernel_drop_count_delta, 3U);
}

TEST(RadarSectorDecoderTest,
     TracksAComplete2048SpokeRevolutionWithoutFalseGaps) {
  RadarSectorDecoder decoder;
  uint16_t scan_number = 0;
  int64_t arrival_ns = 1000;
  size_t decoded_spokes = 0;
  for (uint32_t first = 0; first < 4096; first += 240) {
    std::vector<SpokeSpec> spokes;
    for (uint32_t angle = first; angle < 4096 && spokes.size() < 120;
         angle += 2) {
      spokes.push_back(SpokeSpec{static_cast<uint16_t>(angle), scan_number++});
    }
    const auto packet = makePacket(spokes);
    const auto decoded =
        decoder.decode(packet.data(), packet.size(), metadata(arrival_ns));
    ASSERT_EQ(decoded.size(), 1U);
    EXPECT_EQ(decoded[0].message_type, Sector::MessageType::DATA);
    EXPECT_EQ(decoded[0].preceding_continuity, Sector::MessageType::DATA);
    EXPECT_TRUE(decoded[0].missing_raw_angles.empty());
    ASSERT_FALSE(decoded[0].scanlines.empty());
    EXPECT_EQ(decoded[0].scanlines.front().revolution, 0U);
    EXPECT_EQ(decoded[0].scanlines.back().revolution, 0U);
    decoded_spokes += decoded[0].scanlines.size();
    arrival_ns += 1000;
  }
  EXPECT_EQ(decoded_spokes, 2048U);

  const auto next_revolution = makePacket({SpokeSpec{0, scan_number}});
  const auto decoded = decoder.decode(
      next_revolution.data(), next_revolution.size(), metadata(arrival_ns));
  ASSERT_EQ(decoded.size(), 1U);
  ASSERT_EQ(decoded[0].scanlines.size(), 1U);
  EXPECT_EQ(decoded[0].scanlines[0].revolution, 1U);
  EXPECT_EQ(decoded[0].revolution_start, 1U);
  EXPECT_TRUE(decoded[0].missing_raw_angles.empty());
}

TEST(RadarSectorDecoderTest, ExplicitlyListsAWholeNearRevolutionGap) {
  RadarSectorDecoder decoder;
  const auto first = makePacket({SpokeSpec{0, 1}});
  const auto next = makePacket({SpokeSpec{4094, 2}});
  decoder.decode(first.data(), first.size(), metadata(1000));
  const auto decoded = decoder.decode(next.data(), next.size(), metadata(2000));
  ASSERT_EQ(decoded.size(), 2U);
  ASSERT_EQ(decoded[0].message_type, Sector::MessageType::DATA_MISSING);
  ASSERT_EQ(decoded[0].missing_raw_angles.size(), 2046U);
  EXPECT_EQ(decoded[0].missing_raw_angles.front(), 2U);
  EXPECT_EQ(decoded[0].missing_raw_angles.back(), 4092U);
  EXPECT_EQ(decoded[1].missing_raw_angles, decoded[0].missing_raw_angles);
}

TEST(RadarSectorDecoderTest, MalformedAnglesCannotPoisonContinuityState) {
  RadarSectorDecoder decoder;
  const auto first = makePacket({SpokeSpec{100, 1}, SpokeSpec{102, 2}});
  decoder.decode(first.data(), first.size(), metadata(1000));

  const auto impossible = makePacket({SpokeSpec{5000, 3}});
  auto decoded =
      decoder.decode(impossible.data(), impossible.size(), metadata(2000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  ASSERT_EQ(decoded[0].scanlines.size(), 1U);
  EXPECT_EQ(decoded[0].scanlines[0].raw.angle, 5000U);

  const auto continuous = makePacket({SpokeSpec{104, 4}});
  decoded =
      decoder.decode(continuous.data(), continuous.size(), metadata(3000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::DATA);
  EXPECT_EQ(decoded[0].preceding_continuity, Sector::MessageType::DATA);
  EXPECT_TRUE(decoded[0].missing_raw_angles.empty());
}

TEST(RadarSectorDecoderTest,
     BoundsOversizedCountAndHandlesCounterWrapAndReset) {
  RadarSectorDecoder decoder;
  std::vector<SpokeSpec> oversized;
  for (uint16_t i = 0; i < 121; ++i)
    oversized.push_back(SpokeSpec{static_cast<uint16_t>(2 * i), i});
  const auto packet = makePacket(oversized);
  auto decoded =
      decoder.decode(packet.data(), packet.size(),
                     metadata(1000, std::numeric_limits<uint32_t>::max() - 1));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  EXPECT_EQ(decoded[0].scanlines.size(), 120U);

  const auto next = makePacket({SpokeSpec{240, 121}});
  decoded = decoder.decode(next.data(), next.size(), metadata(2000, 1));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].kernel_drop_count_delta, 3U);

  decoder.reset();
  decoded = decoder.decode(next.data(), next.size(), metadata(3000, 2));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].received_packet_sequence, 0U);
  EXPECT_EQ(decoded[0].previous_arrival_time_ns, 0);
  EXPECT_EQ(decoded[0].kernel_drop_count_delta, 2U);
}

TEST(RadarSectorDecoderTest, RandomMalformedDatagramsAreBoundedAndNeverCrash) {
  RadarSectorDecoder decoder;
  std::mt19937 random(0x48414c4fU);
  std::uniform_int_distribution<size_t> size_distribution(0, 4096);
  std::uniform_int_distribution<unsigned> byte_distribution(0, 255);

  for (uint64_t iteration = 0; iteration < 5000; ++iteration) {
    std::vector<uint8_t> packet(size_distribution(random));
    for (auto &byte : packet)
      byte = static_cast<uint8_t>(byte_distribution(random));
    const auto decoded =
        decoder.decode(packet.empty() ? nullptr : packet.data(), packet.size(),
                       metadata(static_cast<int64_t>(iteration + 1)));
    ASSERT_GE(decoded.size(), 1U);
    ASSERT_LE(decoded.size(), 2U);
    EXPECT_EQ(decoded.back().received_packet_sequence, iteration);
    EXPECT_LE(decoded.back().scanlines.size(), 120U);
    for (const auto &scanline : decoded.back().scanlines)
      EXPECT_EQ(scanline.intensities.size(), 1024U);
  }

  const auto decoded = decoder.decode(nullptr, 65535, metadata(6000));
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].message_type, Sector::MessageType::MALFORMED_PACKET);
  EXPECT_TRUE(decoded[0].scanlines.empty());
}

TEST(RadarUdpIntegrationTest, ReceivesSyntheticMulticastAndDetectsGap) {
  const auto addresses =
      makeAddresses("udp_test", "239.255.42.11", "239.255.42.12");
  CapturingRadar radar(addresses);

  const auto first = makePacket({SpokeSpec{100, 1}, SpokeSpec{102, 2}});
  bool received = false;
  for (int attempt = 0; attempt < 5 && !received; ++attempt) {
    ASSERT_TRUE(sendMulticast(addresses.data, first));
    received = radar.waitFor(1, 300ms);
  }
  ASSERT_TRUE(received) << "loopback multicast packet was not received";

  const auto second = makePacket({SpokeSpec{110, 3}, SpokeSpec{112, 4}});
  ASSERT_TRUE(sendMulticast(addresses.data, second));
  ASSERT_TRUE(radar.waitFor(3, 2s));
  const auto sectors = radar.sectors();
  ASSERT_GE(sectors.size(), 3U);
  EXPECT_GT(sectors[0].arrival_time_ns, 0);
  EXPECT_EQ(sectors[0].source_address, "127.0.0.1");
  EXPECT_EQ(sectors[1].message_type, Sector::MessageType::DATA_MISSING);
  EXPECT_EQ(sectors[1].missing_raw_angles,
            (std::vector<uint16_t>{104, 106, 108}));
  EXPECT_EQ(sectors[2].preceding_continuity, Sector::MessageType::DATA_MISSING);
}

TEST(RadarRosIntegrationTest, PublishesRawLegacyAndEventsAndFlushesJsonl) {
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  const std::string label = "ros_test_" + std::to_string(getpid());
  const auto addresses = makeAddresses(label, "239.255.43.11", "239.255.43.12");
  const auto package_share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("simrad_halo_radar"));
  const auto log_directory = package_share / "logs" / label;
  std::error_code cleanup_error;
  std::filesystem::remove_all(log_directory, cleanup_error);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter(label + ".raw_data_qos_depth", 32),
       rclcpp::Parameter(label + ".event_qos_depth", 32),
       rclcpp::Parameter(label + ".arrival_gap_warning_ms", 100000.0)});
  auto node =
      std::make_shared<rclcpp::Node>("halo_ros_integration_test", options);

  std::vector<simrad_halo_radar::msg::HaloRadarSector> raw_messages;
  std::vector<simrad_halo_radar::msg::HaloRadarEvent> events;
  std::vector<marine_sensor_msgs::msg::RadarSector> legacy_messages;
  auto raw_subscription =
      node->create_subscription<simrad_halo_radar::msg::HaloRadarSector>(
          label + "/raw_data", rclcpp::QoS(32).reliable(),
          [&](simrad_halo_radar::msg::HaloRadarSector::SharedPtr message) {
            raw_messages.push_back(*message);
          });
  auto event_subscription =
      node->create_subscription<simrad_halo_radar::msg::HaloRadarEvent>(
          label + "/events", rclcpp::QoS(32).reliable(),
          [&](simrad_halo_radar::msg::HaloRadarEvent::SharedPtr message) {
            events.push_back(*message);
          });
  auto legacy_subscription =
      node->create_subscription<marine_sensor_msgs::msg::RadarSector>(
          label + "/data", rclcpp::QoS(10),
          [&](marine_sensor_msgs::msg::RadarSector::SharedPtr message) {
            legacy_messages.push_back(*message);
          });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  {
    RosRadar radar(node, addresses);
    const auto first =
        makePacket({SpokeSpec{200, 1, 0x02}, SpokeSpec{202, 2, 0x00}});
    bool got_first = false;
    for (int attempt = 0; attempt < 5 && !got_first; ++attempt) {
      ASSERT_TRUE(sendMulticast(addresses.data, first));
      const auto deadline = std::chrono::steady_clock::now() + 500ms;
      while (std::chrono::steady_clock::now() < deadline &&
             raw_messages.empty())
        executor.spin_some(10ms);
      got_first = !raw_messages.empty();
    }
    ASSERT_TRUE(got_first);

    const auto second = makePacket({SpokeSpec{210, 3}, SpokeSpec{212, 4}});
    ASSERT_TRUE(sendMulticast(addresses.data, second));
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline &&
           (raw_messages.size() < 3 || events.empty() ||
            legacy_messages.size() < 2)) {
      executor.spin_some(10ms);
    }

    ASSERT_GE(raw_messages.size(), 3U);
    ASSERT_GE(events.size(), 1U);
    ASSERT_GE(legacy_messages.size(), 2U);
    EXPECT_EQ(raw_messages[0].radar_model, "Simrad HALO 24");
    EXPECT_EQ(raw_messages[0].radar_serial_number, "SYNTHETIC-HALO24");
    ASSERT_EQ(raw_messages[0].spokes.size(), 2U);
    EXPECT_EQ(raw_messages[0].spokes[0].raw_samples[0], 0xa3U);
    EXPECT_EQ(raw_messages[0].spokes[1].status, 0x00U);
    EXPECT_EQ(raw_messages[1].message_type,
              simrad_halo_radar::msg::HaloRadarSector::DATA_MISSING);
    EXPECT_EQ(raw_messages[1].missing_raw_angles,
              (std::vector<uint16_t>{204, 206, 208}));
    EXPECT_EQ(raw_messages[2].missing_raw_angles,
              (std::vector<uint16_t>{204, 206, 208}));
    EXPECT_EQ(events[0].event_type,
              simrad_halo_radar::msg::HaloRadarEvent::SPOKE_STATUS);
    bool saw_missing_event = false;
    for (const auto &event : events)
      saw_missing_event |= event.event_type ==
                           simrad_halo_radar::msg::HaloRadarEvent::DATA_MISSING;
    EXPECT_TRUE(saw_missing_event);
    ASSERT_EQ(legacy_messages[0].intensities.size(), 1U);
    ASSERT_EQ(legacy_messages[0].intensities[0].echoes.size(), 1024U);
    EXPECT_FLOAT_EQ(legacy_messages[0].intensities[0].echoes[0], 3.0F / 15.0F);
  }
  executor.spin_some();

  ASSERT_TRUE(std::filesystem::exists(log_directory));
  std::vector<std::filesystem::path> logs;
  for (const auto &entry : std::filesystem::directory_iterator(log_directory))
    if (entry.path().extension() == ".jsonl")
      logs.push_back(entry.path());
  ASSERT_EQ(logs.size(), 1U);
  std::ifstream stream(logs.front());
  const std::string contents((std::istreambuf_iterator<char>(stream)),
                             std::istreambuf_iterator<char>());
  EXPECT_NE(contents.find("\"record_type\":\"driver_start\""),
            std::string::npos);
  EXPECT_NE(contents.find("\"record_type\":\"sector\""), std::string::npos);
  EXPECT_NE(contents.find("\"record_type\":\"event\""), std::string::npos);
  EXPECT_NE(contents.find("\"record_type\":\"driver_stop\""),
            std::string::npos);
  EXPECT_NE(contents.find("\"missing_raw_angles\":[204,206,208]"),
            std::string::npos);
}

TEST(RadarRosConfigurationTest, RequiredAuditLogFailureStopsStartup) {
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  const std::string label = "log_failure_test_" + std::to_string(getpid());
  const auto addresses = makeAddresses(label, "239.255.44.11", "239.255.44.12");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter(label + ".event_log_directory",
                         "/proc/simrad_halo_radar_test_logs"),
       rclcpp::Parameter(label + ".require_event_log", true)});
  auto node = std::make_shared<rclcpp::Node>("halo_log_failure_test", options);

  EXPECT_ANY_THROW({ RosRadar radar(node, addresses); });
}
} // namespace
