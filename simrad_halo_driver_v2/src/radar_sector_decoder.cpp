#include "halo_radar.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace simrad_halo_radar {
namespace {
constexpr uint16_t kRawAngleModulus = 4096;
constexpr uint16_t kExpectedRawAngleStep = 2;
constexpr uint16_t kHeadingTrueFlag = 0x4000;
constexpr uint16_t kHeadingValueMask = kRawAngleModulus - 1;
constexpr size_t kSectorHeaderSize = 8;
constexpr size_t kSpokeHeaderSize = 24;
constexpr size_t kMaximumSpokesPerDatagram = 120;

static_assert(sizeof(RawScanline) == 536, "HALO scanline wire size changed");

uint16_t readLe16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(data[1]) << 8;
}

uint32_t readLe32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) | static_cast<uint32_t>(data[1]) << 8 |
         static_cast<uint32_t>(data[2]) << 16 |
         static_cast<uint32_t>(data[3]) << 24;
}

Scanline decodeScanline(const uint8_t *wire) {
  Scanline scanline{};
  std::copy_n(wire, kSpokeHeaderSize, scanline.raw_header.begin());

  scanline.raw.headerLen = wire[0];
  scanline.raw.status = wire[1];
  scanline.raw.scan_number = readLe16(wire + 2);
  scanline.raw.u00 = readLe16(wire + 4);
  scanline.raw.large_range = readLe16(wire + 6);
  scanline.raw.angle = readLe16(wire + 8);
  scanline.raw.heading = readLe16(wire + 10);
  scanline.raw.small_range = readLe16(wire + 12);
  scanline.raw.rotation = readLe16(wire + 14);
  scanline.raw.u02 = readLe32(wire + 16);
  scanline.raw.u03 = readLe32(wire + 20);
  std::copy_n(wire + kSpokeHeaderSize, sizeof(scanline.raw.data),
              scanline.raw.data);

  if (scanline.raw.large_range == 128) {
    scanline.range =
        scanline.raw.small_range == std::numeric_limits<uint16_t>::max()
            ? 0.0F
            : scanline.raw.small_range / 4.0F;
  } else {
    scanline.range = static_cast<float>(scanline.raw.large_range) *
                     static_cast<float>(scanline.raw.small_range) / 512.0F;
  }

  scanline.angle = scanline.raw.angle * 360.0F / kRawAngleModulus;
  scanline.heading_valid =
      (scanline.raw.heading & ~(kHeadingTrueFlag | kHeadingValueMask)) == 0;
  scanline.heading_is_true =
      scanline.heading_valid && (scanline.raw.heading & kHeadingTrueFlag) != 0;
  scanline.ego_heading_raw = scanline.raw.heading & kHeadingValueMask;
  scanline.ego_heading_degrees =
      scanline.ego_heading_raw * 360.0F / kRawAngleModulus;

  scanline.intensities.reserve(1024);
  for (const uint8_t packed : scanline.raw.data) {
    scanline.intensities.push_back(packed & 0x0f);
    scanline.intensities.push_back((packed & 0xf0) >> 4);
  }
  return scanline;
}

void copySyntheticMetadata(const Sector &received, Sector &synthetic) {
  synthetic.arrival_time_ns = received.arrival_time_ns;
  synthetic.previous_arrival_time_ns = received.previous_arrival_time_ns;
  synthetic.arrival_gap_ns = received.arrival_gap_ns;
  synthetic.received_packet_sequence = received.received_packet_sequence;
  synthetic.source_address = received.source_address;
  synthetic.source_port = received.source_port;
  synthetic.socket_message_flags = received.socket_message_flags;
  synthetic.datagram_truncated = received.datagram_truncated;
  synthetic.has_kernel_drop_count = received.has_kernel_drop_count;
  synthetic.kernel_drop_count = received.kernel_drop_count;
  synthetic.kernel_drop_count_delta = received.kernel_drop_count_delta;
  synthetic.expected_raw_angle_step = received.expected_raw_angle_step;
}
} // namespace

void RadarSectorDecoder::reset() { *this = RadarSectorDecoder{}; }

std::vector<Sector>
RadarSectorDecoder::decode(const uint8_t *data, size_t size,
                           DatagramMetadata const &metadata) {
  std::vector<Sector> output;
  Sector sector;
  // Even a malformed datagram with no decodable spokes belongs to the
  // decoder's current revolution for diagnostic accounting.
  sector.revolution_start = m_revolutionCounter;
  sector.revolution_end = m_revolutionCounter;
  sector.arrival_time_ns = metadata.arrival_time_ns;
  sector.previous_arrival_time_ns =
      m_havePreviousArrival ? m_previousArrivalTimeNs : 0;
  sector.arrival_gap_ns = m_havePreviousArrival
                              ? sector.arrival_time_ns - m_previousArrivalTimeNs
                              : 0;
  sector.received_packet_sequence = m_receivedPacketSequence++;
  sector.source_address = metadata.source_address;
  sector.source_port = metadata.source_port;
  sector.datagram_size = static_cast<uint32_t>(
      std::min<size_t>(size, std::numeric_limits<uint32_t>::max()));
  sector.socket_message_flags = metadata.socket_message_flags;
  sector.datagram_truncated = metadata.datagram_truncated;
  sector.has_kernel_drop_count = metadata.has_kernel_drop_count;
  sector.kernel_drop_count = metadata.kernel_drop_count;
  if (metadata.has_kernel_drop_count) {
    sector.kernel_drop_count_delta =
        m_haveKernelDropCount
            ? metadata.kernel_drop_count - m_previousKernelDropCount
            : metadata.kernel_drop_count;
    m_previousKernelDropCount = metadata.kernel_drop_count;
    m_haveKernelDropCount = true;
  }
  sector.expected_raw_angle_step = kExpectedRawAngleStep;

  const size_t header_bytes =
      data == nullptr ? 0 : std::min(size, kSectorHeaderSize);
  if (header_bytes > 0)
    std::copy_n(data, header_bytes, sector.raw_packet_header.begin());

  if (data == nullptr || size < kSectorHeaderSize) {
    sector.message_type = Sector::MessageType::MALFORMED_PACKET;
    output.push_back(std::move(sector));
    m_havePreviousArrival = true;
    m_previousArrivalTimeNs = metadata.arrival_time_ns;
    return output;
  }

  sector.declared_scanline_count = data[5];
  sector.declared_scanline_size = readLe16(data + 6);

  bool malformed = metadata.datagram_truncated ||
                   sector.declared_scanline_size != sizeof(RawScanline) ||
                   sector.declared_scanline_count > kMaximumSpokesPerDatagram;
  size_t available_scanlines = 0;
  if (sector.declared_scanline_size >= sizeof(RawScanline))
    available_scanlines =
        (size - kSectorHeaderSize) / sector.declared_scanline_size;
  const size_t scanline_count =
      std::min({static_cast<size_t>(sector.declared_scanline_count),
                available_scanlines, kMaximumSpokesPerDatagram});
  if (scanline_count != sector.declared_scanline_count)
    malformed = true;

  sector.scanlines.reserve(scanline_count);
  bool angles_valid = true;
  for (size_t i = 0; i < scanline_count; ++i) {
    const uint8_t *wire =
        data + kSectorHeaderSize + i * sector.declared_scanline_size;
    sector.scanlines.push_back(decodeScanline(wire));
    angles_valid &= sector.scanlines.back().raw.angle < kRawAngleModulus;
  }
  malformed |= !angles_valid;

  sector.message_type = malformed ? Sector::MessageType::MALFORMED_PACKET
                                  : Sector::MessageType::DATA;

  if (!sector.scanlines.empty()) {
    sector.has_angle_bounds = true;
    sector.first_raw_angle = sector.scanlines.front().raw.angle;
    sector.last_raw_angle = sector.scanlines.back().raw.angle;
    sector.observed_first_raw_angle = sector.first_raw_angle;
    sector.min_raw_angle = sector.first_raw_angle;
    sector.max_raw_angle = sector.first_raw_angle;
    for (const auto &scanline : sector.scanlines) {
      sector.min_raw_angle = std::min(sector.min_raw_angle, scanline.raw.angle);
      sector.max_raw_angle = std::max(sector.max_raw_angle, scanline.raw.angle);
      if (scanline.heading_valid) {
        if (!sector.has_valid_heading) {
          sector.has_valid_heading = true;
          sector.heading_consistent = true;
          sector.heading_is_true = scanline.heading_is_true;
          sector.ego_heading_raw = scanline.ego_heading_raw;
          sector.ego_heading_degrees = scanline.ego_heading_degrees;
        } else if (sector.heading_is_true != scanline.heading_is_true ||
                   sector.ego_heading_raw != scanline.ego_heading_raw) {
          sector.heading_consistent = false;
        }
      }
    }
    if (angles_valid) {
      for (size_t i = 1; i < sector.scanlines.size(); ++i) {
        const uint16_t previous = sector.scanlines[i - 1].raw.angle;
        const uint16_t current = sector.scanlines[i].raw.angle;
        const uint16_t forward_delta =
            (current + kRawAngleModulus - previous) % kRawAngleModulus;
        if (forward_delta > kExpectedRawAngleStep &&
            forward_delta % kExpectedRawAngleStep == 0) {
          sector.internal_missing_spoke_count +=
              forward_delta / kExpectedRawAngleStep - 1;
        }
      }
    }

    if (!angles_valid) {
      // Preserve every wire spoke for diagnostics, but never let an
      // impossible angle poison continuity or revolution state.
      for (auto &scanline : sector.scanlines)
        scanline.revolution = m_revolutionCounter;
      sector.revolution_start = m_revolutionCounter;
      sector.revolution_end = m_revolutionCounter;
    } else {
      bool first_transition_is_forward = true;
      if (m_havePreviousSpoke) {
        const uint16_t expected =
            (m_previousRawAngle + kExpectedRawAngleStep) % kRawAngleModulus;
        sector.expected_first_raw_angle = expected;
        const uint16_t forward_delta =
            (sector.first_raw_angle + kRawAngleModulus - m_previousRawAngle) %
            kRawAngleModulus;

        if (sector.first_raw_angle != expected) {
          Sector gap;
          copySyntheticMetadata(sector, gap);
          gap.expected_first_raw_angle = expected;
          gap.observed_first_raw_angle = sector.first_raw_angle;

          if (forward_delta > kExpectedRawAngleStep &&
              forward_delta % kExpectedRawAngleStep == 0) {
            gap.message_type = Sector::MessageType::DATA_MISSING;
            uint16_t angle = expected;
            uint16_t previous_angle = m_previousRawAngle;
            uint64_t revolution = m_revolutionCounter;
            bool first_missing = true;
            while (angle != sector.first_raw_angle) {
              if (angle < previous_angle)
                ++revolution;
              if (first_missing) {
                gap.first_raw_angle = angle;
                gap.revolution_start = revolution;
                first_missing = false;
              }
              gap.missing_raw_angles.push_back(angle);
              gap.last_raw_angle = angle;
              gap.revolution_end = revolution;
              previous_angle = angle;
              angle = (angle + kExpectedRawAngleStep) % kRawAngleModulus;
            }
            gap.has_angle_bounds = !gap.missing_raw_angles.empty();
            if (gap.has_angle_bounds) {
              const auto bounds = std::minmax_element(
                  gap.missing_raw_angles.begin(), gap.missing_raw_angles.end());
              gap.min_raw_angle = *bounds.first;
              gap.max_raw_angle = *bounds.second;
            }
          } else {
            gap.message_type = Sector::MessageType::ANGLE_DISCONTINUITY;
            gap.revolution_start = m_revolutionCounter;
            gap.revolution_end = m_revolutionCounter;
            first_transition_is_forward = false;
          }
          sector.preceding_continuity = gap.message_type;
          sector.missing_raw_angles = gap.missing_raw_angles;
          output.push_back(std::move(gap));
        }
      } else {
        sector.expected_first_raw_angle = sector.first_raw_angle;
      }

      uint16_t previous_angle = m_previousRawAngle;
      bool have_previous_angle = m_havePreviousSpoke;
      bool first_scanline = true;
      uint64_t revolution = m_revolutionCounter;
      for (auto &scanline : sector.scanlines) {
        if (have_previous_angle && scanline.raw.angle < previous_angle) {
          const uint16_t forward_delta =
              (scanline.raw.angle + kRawAngleModulus - previous_angle) %
              kRawAngleModulus;
          if (forward_delta > 0 &&
              (!first_scanline || first_transition_is_forward))
            ++revolution;
        }
        scanline.revolution = revolution;
        previous_angle = scanline.raw.angle;
        have_previous_angle = true;
        first_scanline = false;
      }
      sector.revolution_start = sector.scanlines.front().revolution;
      sector.revolution_end = sector.scanlines.back().revolution;
      m_revolutionCounter = revolution;
      m_previousRawAngle = sector.last_raw_angle;
      m_havePreviousSpoke = true;
    }
  }

  output.push_back(std::move(sector));
  m_havePreviousArrival = true;
  m_previousArrivalTimeNs = metadata.arrival_time_ns;
  return output;
}
} // namespace simrad_halo_radar
