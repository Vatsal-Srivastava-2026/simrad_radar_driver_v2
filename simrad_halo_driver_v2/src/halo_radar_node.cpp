#include "halo_radar_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
void assignDuration(builtin_interfaces::msg::Duration &output,
                    int64_t nanoseconds) {
  int64_t seconds = nanoseconds / 1000000000LL;
  int64_t remainder = nanoseconds % 1000000000LL;
  if (remainder < 0) {
    --seconds;
    remainder += 1000000000LL;
  }
  output.sec = static_cast<int32_t>(seconds);
  output.nanosec = static_cast<uint32_t>(remainder);
}

std::string jsonEscape(const std::string &input) {
  std::ostringstream output;
  for (const char c : input) {
    switch (c) {
    case '\\':
      output << "\\\\";
      break;
    case '"':
      output << "\\\"";
      break;
    case '\n':
      output << "\\n";
      break;
    case '\r':
      output << "\\r";
      break;
    case '\t':
      output << "\\t";
      break;
    default:
      output << c;
      break;
    }
  }
  return output.str();
}
} // namespace

RosRadar::RosRadar(rclcpp::Node::SharedPtr node,
                   simrad_halo_radar::AddressSet const &addresses)
    : simrad_halo_radar::Radar(addresses) {
  this->node_ = node;
  m_radar_id = addresses.label;
  m_radar_serial_number = addresses.serial_number;
  m_radar_model = node_->declare_parameter<std::string>(
      addresses.label + ".radar_model", "Simrad HALO 24");

  node_->declare_parameter(addresses.label + ".range_correction_factor",
                           this->m_rangeCorrectionFactor);
  node_->get_parameter(addresses.label + ".range_correction_factor",
                       this->m_rangeCorrectionFactor);

  node_->declare_parameter(addresses.label + ".frame_id", this->m_frame_id);
  node_->get_parameter(addresses.label + ".frame_id", this->m_frame_id);

  this->m_data_pub =
      node_->create_publisher<marine_sensor_msgs::msg::RadarSector>(
          addresses.label + "/data", 10);
  const auto raw_qos_depth_parameter = addresses.label + ".raw_data_qos_depth";
  const auto configured_raw_qos_depth =
      node_->declare_parameter<int64_t>(raw_qos_depth_parameter, 512);
  const size_t raw_qos_depth =
      static_cast<size_t>(std::max<int64_t>(1, configured_raw_qos_depth));
  const auto raw_qos = rclcpp::QoS(rclcpp::KeepLast(raw_qos_depth))
                           .reliable()
                           .durability_volatile();
  this->m_raw_data_pub =
      node_->create_publisher<simrad_halo_driver_v2::msg::HaloRadarSector>(
          addresses.label + "/raw_data", raw_qos);

  m_enable_event_logging = node_->declare_parameter<bool>(
      addresses.label + ".enable_event_logging", true);
  m_require_event_log = node_->declare_parameter<bool>(
      addresses.label + ".require_event_log", true);
  const auto event_qos_depth = static_cast<size_t>(
      std::max<int64_t>(1, node_->declare_parameter<int64_t>(
                               addresses.label + ".event_qos_depth", 256)));
  const auto event_qos = rclcpp::QoS(rclcpp::KeepLast(event_qos_depth))
                             .reliable()
                             .durability_volatile();
  this->m_event_pub =
      node_->create_publisher<simrad_halo_driver_v2::msg::HaloRadarEvent>(
          addresses.label + "/events", event_qos);
  const double arrival_gap_warning_ms = node_->declare_parameter<double>(
      addresses.label + ".arrival_gap_warning_ms", 100.0);
  m_arrival_gap_warning_ns =
      static_cast<int64_t>(std::max(0.0, arrival_gap_warning_ms) * 1000000.0);

  const auto package_log_directory =
      std::filesystem::path(ament_index_cpp::get_package_share_directory(
          "simrad_halo_driver_v2")) /
      "logs";
  const auto event_log_directory = node_->declare_parameter<std::string>(
      addresses.label + ".event_log_directory", package_log_directory.string());
  if (m_enable_event_logging) {
    try {
      const auto radar_log_directory =
          std::filesystem::path(event_log_directory) / addresses.label;
      std::filesystem::create_directories(radar_log_directory);
      const auto now = std::chrono::system_clock::now();
      const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
      std::tm utc{};
      gmtime_r(&now_time, &utc);
      std::ostringstream filename;
      filename << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << "_events.jsonl";
      m_event_log_path = (radar_log_directory / filename.str()).string();
      m_event_log_file.open(m_event_log_path, std::ios::out | std::ios::app);
      if (!m_event_log_file) {
        const std::string error =
            addresses.label + " could not open event log " + m_event_log_path;
        if (m_require_event_log)
          throw std::runtime_error(error);
        RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
      } else {
        m_event_log_file << "{\"record_type\":\"driver_start\",\"radar\":\""
                         << jsonEscape(addresses.label)
                         << "\",\"radar_model\":\"" << jsonEscape(m_radar_model)
                         << "\",\"radar_serial_number\":\""
                         << jsonEscape(m_radar_serial_number)
                         << "\",\"schema_version\":1}\n";
        m_event_log_file.flush();
        RCLCPP_INFO(node_->get_logger(), "%s audit log: %s",
                    addresses.label.c_str(), m_event_log_path.c_str());
      }
    } catch (const std::exception &exception) {
      if (m_require_event_log)
        throw;
      RCLCPP_ERROR(node_->get_logger(), "%s could not create event log: %s",
                   addresses.label.c_str(), exception.what());
    }
  }
  this->m_state_pub =
      node_->create_publisher<marine_radar_control_msgs::msg::RadarControlSet>(
          addresses.label + "/state", 10);
  this->m_state_change_sub = node_->create_subscription<
      marine_radar_control_msgs::msg::RadarControlValue>(
      addresses.label + "/change_state", 10,
      std::bind(&RosRadar::stateChangeCallback, this, _1));

  m_heartbeatTimer = node_->create_wall_timer(
      std::chrono::seconds(1), std::bind(&RosRadar::hbTimerCallback, this));

  startThreads();

  RCLCPP_INFO(node_->get_logger(), "%s subnode started", m_radar_id.c_str());
}

RosRadar::~RosRadar() {
  // Stop callbacks before closing members used by the receive thread.
  stopThreads();
  if (m_event_log_file) {
    const std::lock_guard<std::mutex> lock(m_event_log_mutex);
    m_event_log_file << "{\"record_type\":\"driver_stop\",\"radar\":\""
                     << jsonEscape(m_radar_id) << "\"}\n";
    m_event_log_file.flush();
  }
  RCLCPP_INFO(node_->get_logger(), "%s subnode stopped", m_radar_id.c_str());
}

void RosRadar::processData(simrad_halo_radar::Sector const &sector) {
  simrad_halo_driver_v2::msg::HaloRadarSector raw_sector;
  raw_sector.header.stamp = rclcpp::Time(sector.arrival_time_ns);
  raw_sector.header.frame_id = m_frame_id;
  raw_sector.message_type = static_cast<uint8_t>(sector.message_type);
  raw_sector.radar_id = m_radar_id;
  raw_sector.radar_model = m_radar_model;
  raw_sector.radar_serial_number = m_radar_serial_number;
  raw_sector.preceding_continuity =
      static_cast<uint8_t>(sector.preceding_continuity);
  raw_sector.received_packet_sequence = sector.received_packet_sequence;
  if (sector.previous_arrival_time_ns > 0)
    raw_sector.previous_packet_stamp =
        rclcpp::Time(sector.previous_arrival_time_ns);
  assignDuration(raw_sector.arrival_gap, sector.arrival_gap_ns);
  raw_sector.source_address = sector.source_address;
  raw_sector.source_port = sector.source_port;
  raw_sector.datagram_size = sector.datagram_size;
  raw_sector.socket_message_flags = sector.socket_message_flags;
  raw_sector.datagram_truncated = sector.datagram_truncated;
  raw_sector.has_kernel_drop_count = sector.has_kernel_drop_count;
  raw_sector.kernel_drop_count = sector.kernel_drop_count;
  raw_sector.kernel_drop_count_delta = sector.kernel_drop_count_delta;
  raw_sector.raw_packet_header = sector.raw_packet_header;
  raw_sector.declared_scanline_count = sector.declared_scanline_count;
  raw_sector.declared_scanline_size = sector.declared_scanline_size;
  raw_sector.parsed_scanline_count =
      static_cast<uint16_t>(sector.scanlines.size());
  raw_sector.has_angle_bounds = sector.has_angle_bounds;
  raw_sector.min_raw_angle = sector.min_raw_angle;
  raw_sector.max_raw_angle = sector.max_raw_angle;
  raw_sector.first_raw_angle = sector.first_raw_angle;
  raw_sector.last_raw_angle = sector.last_raw_angle;
  raw_sector.observed_first_raw_angle = sector.observed_first_raw_angle;
  raw_sector.expected_first_raw_angle = sector.expected_first_raw_angle;
  raw_sector.expected_raw_angle_step = sector.expected_raw_angle_step;
  raw_sector.has_valid_heading = sector.has_valid_heading;
  raw_sector.heading_consistent = sector.heading_consistent;
  raw_sector.heading_is_true = sector.heading_is_true;
  raw_sector.ego_heading_raw = sector.ego_heading_raw;
  raw_sector.ego_heading_degrees = sector.ego_heading_degrees;
  raw_sector.revolution_start = sector.revolution_start;
  raw_sector.revolution_end = sector.revolution_end;
  raw_sector.missing_raw_angles = sector.missing_raw_angles;
  raw_sector.spokes.reserve(sector.scanlines.size());
  for (const auto &scanline : sector.scanlines) {
    simrad_halo_driver_v2::msg::HaloRadarSpoke spoke;
    spoke.raw_header = scanline.raw_header;
    spoke.header_length = scanline.raw.headerLen;
    spoke.status = scanline.raw.status;
    spoke.scan_number = scanline.raw.scan_number;
    spoke.marker = scanline.raw.u00;
    spoke.large_range = scanline.raw.large_range;
    spoke.raw_angle = scanline.raw.angle;
    spoke.raw_heading = scanline.raw.heading;
    spoke.heading_valid = scanline.heading_valid;
    spoke.heading_is_true = scanline.heading_is_true;
    spoke.ego_heading_raw = scanline.ego_heading_raw;
    spoke.ego_heading_degrees = scanline.ego_heading_degrees;
    spoke.small_range = scanline.raw.small_range;
    spoke.rotation = scanline.raw.rotation;
    spoke.unknown_02 = scanline.raw.u02;
    spoke.unknown_03 = scanline.raw.u03;
    std::copy(std::begin(scanline.raw.data), std::end(scanline.raw.data),
              spoke.raw_samples.begin());
    spoke.revolution = scanline.revolution;
    raw_sector.spokes.push_back(std::move(spoke));
  }
  writeSectorLog(sector);
  logSectorEvents(sector);
  this->m_raw_data_pub->publish(raw_sector);

  if (sector.message_type ==
          simrad_halo_radar::Sector::MessageType::DATA_MISSING ||
      sector.message_type ==
          simrad_halo_radar::Sector::MessageType::ANGLE_DISCONTINUITY ||
      sector.scanlines.empty())
    return;

  std::vector<const simrad_halo_radar::Scanline *> scanlines;
  scanlines.reserve(sector.scanlines.size());
  for (const auto &scanline : sector.scanlines)
    if (scanline.raw.status == 2)
      scanlines.push_back(&scanline);
  if (scanlines.empty())
    return;

  marine_sensor_msgs::msg::RadarSector rs;
  rs.header.stamp = rclcpp::Time(sector.arrival_time_ns);
  rs.header.frame_id = m_frame_id;
  rs.angle_start = 2.0 * M_PI * (360 - scanlines.front()->angle) / 360.0;
  double angle_max = 2.0 * M_PI * (360 - scanlines.back()->angle) / 360.0;
  if (scanlines.size() > 1) {
    if (angle_max > rs.angle_start &&
        angle_max - rs.angle_start >
            M_PI) // have we looped around (also make sure angle are decreasing)
      angle_max -= 2.0 * M_PI;
    rs.angle_increment =
        (angle_max - rs.angle_start) / double(scanlines.size() - 1);
  }
  rs.range_min = 0.0;
  rs.range_max = scanlines.front()->range;
  for (auto sl : scanlines) {
    marine_sensor_msgs::msg::RadarEcho echo;
    for (auto i : sl->intensities)
      echo.echoes.push_back(i / 15.0); // 4 bit int to float
    rs.intensities.push_back(echo);
  }

  auto angular_speed = m_estimator.update(rs.header.stamp, rs.angle_start);
  double scan_time = 0.0;
  if (angular_speed != 0.0) {
    scan_time = 2 * M_PI / fabs(angular_speed);
    RCLCPP_DEBUG(this->node_->get_logger(), "scan time updated: %.4f\n",
                 scan_time);
  }

  rs.scan_time = rclcpp::Duration::from_seconds(scan_time);

  double time_increment = 0.0;
  if (scan_time > 0)
    time_increment = std::abs(rs.angle_increment) / scan_time;
  rs.time_increment = rclcpp::Duration::from_seconds(time_increment);

  this->m_data_pub->publish(rs);
}

void RosRadar::writeSectorLog(simrad_halo_radar::Sector const &sector) {
  if (!m_enable_event_logging || !m_event_log_file)
    return;

  std::map<uint8_t, uint16_t> status_counts;
  for (const auto &scanline : sector.scanlines)
    ++status_counts[scanline.raw.status];

  const std::lock_guard<std::mutex> lock(m_event_log_mutex);
  m_event_log_file
      << "{\"record_type\":\"sector\""
      << ",\"radar\":\"" << jsonEscape(m_radar_id) << "\""
      << ",\"radar_model\":\"" << jsonEscape(m_radar_model) << "\""
      << ",\"radar_serial_number\":\"" << jsonEscape(m_radar_serial_number)
      << "\""
      << ",\"arrival_time_ns\":" << sector.arrival_time_ns
      << ",\"previous_arrival_time_ns\":" << sector.previous_arrival_time_ns
      << ",\"arrival_gap_ns\":" << sector.arrival_gap_ns
      << ",\"message_type\":" << static_cast<unsigned>(sector.message_type)
      << ",\"preceding_continuity\":"
      << static_cast<unsigned>(sector.preceding_continuity)
      << ",\"received_packet_sequence\":" << sector.received_packet_sequence
      << ",\"source_address\":\"" << jsonEscape(sector.source_address) << "\""
      << ",\"source_port\":" << sector.source_port
      << ",\"datagram_size\":" << sector.datagram_size
      << ",\"socket_message_flags\":" << sector.socket_message_flags
      << ",\"datagram_truncated\":"
      << (sector.datagram_truncated ? "true" : "false")
      << ",\"declared_scanline_count\":"
      << static_cast<unsigned>(sector.declared_scanline_count)
      << ",\"declared_scanline_size\":" << sector.declared_scanline_size
      << ",\"parsed_scanline_count\":" << sector.scanlines.size()
      << ",\"has_angle_bounds\":"
      << (sector.has_angle_bounds ? "true" : "false")
      << ",\"min_raw_angle\":" << sector.min_raw_angle
      << ",\"max_raw_angle\":" << sector.max_raw_angle
      << ",\"first_raw_angle\":" << sector.first_raw_angle
      << ",\"last_raw_angle\":" << sector.last_raw_angle
      << ",\"observed_first_raw_angle\":" << sector.observed_first_raw_angle
      << ",\"expected_first_raw_angle\":" << sector.expected_first_raw_angle
      << ",\"expected_raw_angle_step\":" << sector.expected_raw_angle_step
      << ",\"has_valid_heading\":"
      << (sector.has_valid_heading ? "true" : "false")
      << ",\"heading_consistent\":"
      << (sector.heading_consistent ? "true" : "false")
      << ",\"heading_is_true\":" << (sector.heading_is_true ? "true" : "false")
      << ",\"ego_heading_raw\":" << sector.ego_heading_raw
      << ",\"ego_heading_degrees\":" << sector.ego_heading_degrees
      << ",\"revolution_start\":" << sector.revolution_start
      << ",\"revolution_end\":" << sector.revolution_end
      << ",\"has_kernel_drop_count\":"
      << (sector.has_kernel_drop_count ? "true" : "false")
      << ",\"kernel_drop_count\":" << sector.kernel_drop_count
      << ",\"kernel_drop_count_delta\":" << sector.kernel_drop_count_delta
      << ",\"missing_raw_angles\":[";
  for (size_t i = 0; i < sector.missing_raw_angles.size(); ++i) {
    if (i > 0)
      m_event_log_file << ',';
    m_event_log_file << sector.missing_raw_angles[i];
  }
  m_event_log_file << "],\"spoke_status_counts\":{";
  bool first = true;
  for (const auto &entry : status_counts) {
    if (!first)
      m_event_log_file << ',';
    m_event_log_file << '"' << static_cast<unsigned>(entry.first)
                     << "\":" << entry.second;
    first = false;
  }
  m_event_log_file << "}}\n";
  m_event_log_file.flush();
}

void RosRadar::writeEventLog(
    simrad_halo_driver_v2::msg::HaloRadarEvent const &event) {
  if (!m_enable_event_logging || !m_event_log_file)
    return;

  const int64_t timestamp_ns =
      static_cast<int64_t>(event.header.stamp.sec) * 1000000000LL +
      event.header.stamp.nanosec;
  const int64_t arrival_gap_ns =
      static_cast<int64_t>(event.arrival_gap.sec) * 1000000000LL +
      event.arrival_gap.nanosec;
  const std::lock_guard<std::mutex> lock(m_event_log_mutex);
  m_event_log_file << "{\"record_type\":\"event\""
                   << ",\"radar\":\"" << jsonEscape(m_radar_id) << "\""
                   << ",\"radar_model\":\"" << jsonEscape(m_radar_model) << "\""
                   << ",\"radar_serial_number\":\""
                   << jsonEscape(m_radar_serial_number) << "\""
                   << ",\"timestamp_ns\":" << timestamp_ns << ",\"event_type\":"
                   << static_cast<unsigned>(event.event_type)
                   << ",\"received_packet_sequence\":"
                   << event.received_packet_sequence
                   << ",\"revolution_start\":" << event.revolution_start
                   << ",\"revolution_end\":" << event.revolution_end
                   << ",\"arrival_gap_ns\":" << arrival_gap_ns
                   << ",\"previous_raw_angle\":" << event.previous_raw_angle
                   << ",\"expected_raw_angle\":" << event.expected_raw_angle
                   << ",\"observed_raw_angle\":" << event.observed_raw_angle
                   << ",\"kernel_drop_count\":" << event.kernel_drop_count
                   << ",\"kernel_drop_count_delta\":"
                   << event.kernel_drop_count_delta << ",\"details\":\""
                   << jsonEscape(event.details) << "\""
                   << ",\"affected_raw_angles\":[";
  for (size_t i = 0; i < event.affected_raw_angles.size(); ++i) {
    if (i > 0)
      m_event_log_file << ',';
    m_event_log_file << event.affected_raw_angles[i];
  }
  m_event_log_file << "],\"spoke_statuses\":[";
  for (size_t i = 0; i < event.spoke_statuses.size(); ++i) {
    if (i > 0)
      m_event_log_file << ',';
    m_event_log_file << static_cast<unsigned>(event.spoke_statuses[i]);
  }
  m_event_log_file << "],\"spoke_status_counts\":[";
  for (size_t i = 0; i < event.spoke_status_counts.size(); ++i) {
    if (i > 0)
      m_event_log_file << ',';
    m_event_log_file << event.spoke_status_counts[i];
  }
  m_event_log_file << "]}\n";
  m_event_log_file.flush();
}

void RosRadar::publishEvent(uint8_t event_type,
                            simrad_halo_radar::Sector const &sector,
                            std::string const &details,
                            std::vector<uint8_t> const &statuses,
                            std::vector<uint16_t> const &status_counts) {
  if (!m_enable_event_logging)
    return;

  simrad_halo_driver_v2::msg::HaloRadarEvent event;
  event.header.stamp = rclcpp::Time(sector.arrival_time_ns);
  event.header.frame_id = m_frame_id;
  event.event_type = event_type;
  event.radar_id = m_radar_id;
  event.radar_model = m_radar_model;
  event.radar_serial_number = m_radar_serial_number;
  event.received_packet_sequence = sector.received_packet_sequence;
  event.revolution_start = sector.revolution_start;
  event.revolution_end = sector.revolution_end;
  assignDuration(event.arrival_gap, sector.arrival_gap_ns);
  event.previous_raw_angle = (sector.expected_first_raw_angle + 4096 -
                              sector.expected_raw_angle_step) %
                             4096;
  event.expected_raw_angle = sector.expected_first_raw_angle;
  event.observed_raw_angle = sector.observed_first_raw_angle;
  event.affected_raw_angles = sector.missing_raw_angles;
  event.kernel_drop_count = sector.kernel_drop_count;
  event.kernel_drop_count_delta = sector.kernel_drop_count_delta;
  event.spoke_statuses = statuses;
  event.spoke_status_counts = status_counts;
  event.details = details;
  writeEventLog(event);
  m_event_pub->publish(event);
}

void RosRadar::logSectorEvents(simrad_halo_radar::Sector const &sector) {
  if (!m_enable_event_logging)
    return;

  using Event = simrad_halo_driver_v2::msg::HaloRadarEvent;
  if (sector.message_type ==
      simrad_halo_radar::Sector::MessageType::DATA_MISSING) {
    std::ostringstream details;
    details << sector.missing_raw_angles.size()
            << " spokes missing before raw angle "
            << sector.observed_first_raw_angle;
    RCLCPP_WARN(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                details.str().c_str());
    publishEvent(Event::DATA_MISSING, sector, details.str());
  } else if (sector.message_type ==
             simrad_halo_radar::Sector::MessageType::ANGLE_DISCONTINUITY) {
    std::ostringstream details;
    details << "ambiguous spoke transition: expected "
            << sector.expected_first_raw_angle << ", observed "
            << sector.observed_first_raw_angle;
    RCLCPP_WARN(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                details.str().c_str());
    publishEvent(Event::ANGLE_DISCONTINUITY, sector, details.str());
  }

  // The remaining events correspond only to a real received UDP datagram.
  if (sector.datagram_size == 0)
    return;

  if (sector.message_type ==
      simrad_halo_radar::Sector::MessageType::MALFORMED_PACKET) {
    std::ostringstream details;
    details << "malformed UDP sector: " << sector.datagram_size
            << " bytes, declared "
            << static_cast<unsigned>(sector.declared_scanline_count)
            << " spokes of " << sector.declared_scanline_size
            << " bytes, parsed " << sector.scanlines.size();
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                 details.str().c_str());
    publishEvent(Event::MALFORMED_PACKET, sector, details.str());
  }

  if (sector.has_kernel_drop_count && sector.kernel_drop_count_delta > 0) {
    std::ostringstream details;
    details << sector.kernel_drop_count_delta
            << " new kernel UDP receive-queue drops (total "
            << sector.kernel_drop_count << ')';
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                 details.str().c_str());
    publishEvent(Event::KERNEL_DROP, sector, details.str());
  }

  if (sector.previous_arrival_time_ns > 0 &&
      sector.arrival_gap_ns >= m_arrival_gap_warning_ns) {
    std::ostringstream details;
    details << "UDP arrival gap " << (sector.arrival_gap_ns / 1000000.0)
            << " ms";
    RCLCPP_WARN(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                details.str().c_str());
    publishEvent(Event::ARRIVAL_GAP, sector, details.str());
  }

  std::map<uint8_t, uint16_t> nonstandard_status_counts;
  for (const auto &scanline : sector.scanlines)
    if (scanline.raw.status != 0x02)
      ++nonstandard_status_counts[scanline.raw.status];
  if (!nonstandard_status_counts.empty()) {
    std::vector<uint8_t> statuses;
    std::vector<uint16_t> counts;
    std::ostringstream details;
    details << "non-0x02 spoke statuses:";
    for (const auto &entry : nonstandard_status_counts) {
      statuses.push_back(entry.first);
      counts.push_back(entry.second);
      details << " 0x" << std::hex << static_cast<unsigned>(entry.first)
              << std::dec << '=' << entry.second;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: %s", m_radar_id.c_str(),
                details.str().c_str());
    publishEvent(Event::SPOKE_STATUS, sector, details.str(), statuses, counts);
  }

  if (!m_have_event_revolution) {
    m_last_event_revolution = sector.revolution_start;
    m_have_event_revolution = true;
  }
  if (sector.revolution_end > m_last_event_revolution) {
    for (uint64_t completed = m_last_event_revolution;
         completed < sector.revolution_end; ++completed) {
      auto completed_sector = sector;
      completed_sector.revolution_start = completed;
      completed_sector.revolution_end = completed;
      std::ostringstream details;
      details << "revolution " << completed << " completed";
      publishEvent(Event::REVOLUTION_COMPLETED, completed_sector,
                   details.str());
    }
    m_last_event_revolution = sector.revolution_end;
  }
}

void RosRadar::stateUpdated() {
  marine_radar_control_msgs::msg::RadarControlSet rcs;

  std::string statusEnums[] = {"standby", "transmit", ""};

  createEnumControl("status", "Status", statusEnums, rcs);
  createFloatControl("range", "Range", 25, 75000, rcs);

  std::string modeEnums[] = {"custom",  "harbor", "offshore",
                             "weather", "bird",   ""};

  createEnumControl("mode", "Mode", modeEnums, rcs);
  createFloatWithAutoControl("gain", "gain_mode", "Gain", 0, 100, rcs);
  createFloatWithAutoControl("sea_clutter", "sea_clutter_mode", "Sea clutter",
                             0, 100, rcs);
  createFloatControl("auto_sea_clutter_nudge", "Auto sea clut adj", -50, 50,
                     rcs);

  std::string seaStateEnums[] = {"calm", "moderate", "rough", ""};

  createEnumControl("sea_state", "Sea state", seaStateEnums, rcs);
  createFloatControl("rain_clutter", "Rain clutter", 0, 100, rcs);

  std::string lowMedHighEnums[] = {"off", "low", "medium", "high", ""};

  createEnumControl("noise_rejection", "Noise rejection", lowMedHighEnums, rcs);
  createEnumControl("target_expansion", "Target expansion", lowMedHighEnums,
                    rcs);
  createEnumControl("interference_rejection", "Interf. rej", lowMedHighEnums,
                    rcs);
  createEnumControl("target_separation", "Target separation", lowMedHighEnums,
                    rcs);

  std::string scanSpeedEnums[] = {"off", "medium", "high", ""};

  createEnumControl("scan_speed", "Fast scan", scanSpeedEnums, rcs);

  std::string dopplerModeEnums[] = {"off", "normal", "approaching_only", ""};

  createEnumControl("doppler_mode", "VelocityTrack", dopplerModeEnums, rcs);
  createFloatControl("doppler_speed", "Speed threshold", 0.05, 15.95, rcs);
  createFloatControl("antenna_height", "Antenna height", 0.0, 30.175, rcs);
  createFloatControl("bearing_alignment", "Bearing alignment", 0, 360, rcs);
  createFloatWithAutoControl("sidelobe_suppression",
                             "sidelobe_suppression_mode", "Sidelobe sup.", 0,
                             100, rcs);
  createEnumControl("lights", "Halo light", lowMedHighEnums, rcs);

  this->m_state_pub->publish(rcs);
}

void RosRadar::stateChangeCallback(
    const marine_radar_control_msgs::msg::RadarControlValue::SharedPtr cv) {
  sendCommand(cv->key, cv->value);
}

void RosRadar::hbTimerCallback() {
  if (checkHeartbeat())
    stateUpdated();
}

void RosRadar::createEnumControl(
    std::string const &name, std::string const &label,
    std::string const enums[],
    marine_radar_control_msgs::msg::RadarControlSet &rcs) {
  if (m_state.find(name) != m_state.end()) {
    marine_radar_control_msgs::msg::RadarControlItem rci;
    rci.name = name;
    rci.value = m_state[name];
    rci.label = label;
    rci.type =
        marine_radar_control_msgs::msg::RadarControlItem::CONTROL_TYPE_ENUM;
    for (int i = 0; !enums[i].empty(); i++)
      rci.enums.push_back(enums[i]);
    rcs.items.push_back(rci);
  }
}

void RosRadar::createFloatControl(
    std::string const &name, std::string const &label, float min_value,
    float max_value, marine_radar_control_msgs::msg::RadarControlSet &rcs) {
  if (m_state.find(name) != m_state.end()) {
    marine_radar_control_msgs::msg::RadarControlItem rci;
    rci.name = name;
    rci.value = m_state[name];
    rci.label = label;
    rci.type =
        marine_radar_control_msgs::msg::RadarControlItem::CONTROL_TYPE_FLOAT;
    rci.min_value = min_value;
    rci.max_value = max_value;
    rcs.items.push_back(rci);
  }
}

void RosRadar::createFloatWithAutoControl(
    std::string const &name, std::string const &auto_name,
    std::string const &label, float min_value, float max_value,
    marine_radar_control_msgs::msg::RadarControlSet &rcs) {
  if (m_state.find(name) != m_state.end() &&
      m_state.find(auto_name) != m_state.end()) {
    marine_radar_control_msgs::msg::RadarControlItem rci;
    rci.name = name;
    std::string value = m_state[name];
    if (m_state[auto_name] == "auto")
      value = "auto";
    rci.value = value;
    rci.label = label;
    rci.type = marine_radar_control_msgs::msg::RadarControlItem::
        CONTROL_TYPE_FLOAT_WITH_AUTO;
    rci.min_value = min_value;
    rci.max_value = max_value;
    rcs.items.push_back(rci);
  }
}
