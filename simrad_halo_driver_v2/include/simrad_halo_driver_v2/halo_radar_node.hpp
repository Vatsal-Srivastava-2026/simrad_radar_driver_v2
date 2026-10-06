#ifndef HALO_RADAR_NODE_HPP
#define HALO_RADAR_NODE_HPP

#include "angular_speed_estimator.h"
#include "halo_radar.h"
#include "marine_radar_control_msgs/msg/radar_control_item.hpp"
#include "marine_radar_control_msgs/msg/radar_control_set.hpp"
#include "marine_radar_control_msgs/msg/radar_control_value.hpp"
#include "marine_sensor_msgs/msg/radar_echo.hpp"
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "simrad_halo_driver_v2/msg/halo_radar_event.hpp"
#include "simrad_halo_driver_v2/msg/halo_radar_sector.hpp"
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

using std::placeholders::_1;

class RosRadar : public simrad_halo_radar::Radar {
public:
  /*!
   * \brief Creates a RosRadar object for publishing Simrad Halo Radar data
   * \param node SharedPtr to ros2 node,
   * \param addresses vector of IP addresses of radars
   */
  RosRadar(rclcpp::Node::SharedPtr node,
           simrad_halo_radar::AddressSet const &addresses);
  ~RosRadar();

protected:
  /*!
   * \brief Process incoming scanlines from radar and publish RadarSector
   * \param scanlines A radar scanline containing angle, range, and a vector of
   * intensities
   */
  void processData(simrad_halo_radar::Sector const &sector) override;

  /*!
   * \brief Publishes radar state on heartbeat.
   */
  void stateUpdated() override;

private:
  struct RevolutionStatistics {
    uint32_t received_sector_count = 0;
    uint32_t received_spoke_count = 0;
    uint32_t missing_sector_gap_count = 0;
    uint32_t missing_spoke_count = 0;
    uint32_t internal_missing_spoke_count = 0;
    uint32_t invalid_sector_count = 0;
    uint32_t ambiguous_transition_count = 0;
    uint32_t invalid_spoke_count = 0;
  };

  /*!
   * \brief Send command to radar to update state upon receiving a change state
   * command. \param cv RadarControlValue
   */
  void stateChangeCallback(
      const marine_radar_control_msgs::msg::RadarControlValue::SharedPtr cv);

  /*!
   * \brief Publish radar state for every heartbeat.
   */
  void hbTimerCallback();

  /*!
   * \brief Update RadarControlSet message with enum control
   * \param name Name of RadarControlItem
   * \param label Label of RadarControlItem
   * \param enums[] Array of enums
   * \param rcs RadarControlSet message to update
   */
  void createEnumControl(std::string const &name, std::string const &label,
                         std::string const enums[],
                         marine_radar_control_msgs::msg::RadarControlSet &rcs);

  /*!
   * \brief Update RadarControlSet message with float control
   * \param name Name of RadarControlItem
   * \param label Label of RadarControlItem
   * \param min_value Minimum value of control parameter
   * \param max_value Maximum value of control value
   * \param rcs RadarControlSet to update
   */
  void createFloatControl(std::string const &name, std::string const &label,
                          float min_value, float max_value,
                          marine_radar_control_msgs::msg::RadarControlSet &rcs);

  /*!
   * \brief Update RadarControlSet message with float with auto control
   * \param name Name of RadarControlItem
   * \param auto_name
   * \param label Label of RadarControlItem
   * \param min_value Minimum value of control parameter
   * \param max_value Maximum value of control parameter
   * \param rcs RadarControlSet to update
   */
  void createFloatWithAutoControl(
      std::string const &name, std::string const &auto_name,
      std::string const &label, float min_value, float max_value,
      marine_radar_control_msgs::msg::RadarControlSet &rcs);
  void logSectorEvents(simrad_halo_radar::Sector const &sector);
  void publishEvent(uint8_t event_type, simrad_halo_radar::Sector const &sector,
                    std::string const &details,
                    std::vector<uint8_t> const &statuses = {},
                    std::vector<uint16_t> const &status_counts = {},
                    uint32_t event_count = 0);
  void writeSectorLog(simrad_halo_radar::Sector const &sector);
  void writeEventLog(simrad_halo_driver_v2::msg::HaloRadarEvent const &event,
                     uint32_t event_count);
  void updateRevolutionStatistics(
      simrad_halo_radar::Sector const &sector);
  void reportCompletedRevolutions(
      simrad_halo_radar::Sector const &sector);
  void writeRevolutionSummary(
      uint64_t revolution, int64_t timestamp_ns,
      RevolutionStatistics const &statistics, bool partial_revolution);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr m_data_pub;
  rclcpp::Publisher<simrad_halo_driver_v2::msg::HaloRadarSector>::SharedPtr
      m_raw_data_pub;
  rclcpp::Publisher<simrad_halo_driver_v2::msg::HaloRadarEvent>::SharedPtr
      m_event_pub;
  rclcpp::Publisher<marine_radar_control_msgs::msg::RadarControlSet>::SharedPtr
      m_state_pub;
  rclcpp::Subscription<marine_radar_control_msgs::msg::RadarControlValue>::
      SharedPtr m_state_change_sub;
  rclcpp::TimerBase::SharedPtr m_heartbeatTimer;

  double m_rangeCorrectionFactor = 1.024;
  std::string m_frame_id = "radar_link";
  AngularSpeedEstimator m_estimator;
  std::string m_radar_id;
  std::string m_radar_model = "Simrad HALO 24";
  std::string m_radar_serial_number;
  bool m_enable_event_logging = true;
  bool m_require_event_log = true;
  int64_t m_arrival_gap_warning_ns = 100000000;
  bool m_have_event_revolution = false;
  uint64_t m_last_event_revolution = 0;
  uint64_t m_first_event_revolution = 0;
  std::map<uint64_t, RevolutionStatistics> m_revolution_statistics;
  std::ofstream m_event_log_file;
  std::mutex m_event_log_mutex;
  std::string m_event_log_path;
};

#endif
