# ROS Driver for the Simrad Halo series of marine radars

This driver interfaces with Simrad HALO radar via multicast UDP communication.

## Installation

Clone this repo into your workspace.

Source ROS 2 before invoking `colcon`:

```bash
source /opt/ros/humble/setup.bash
```

Install dependencies registered with rosdep:

```bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

In particular, the legacy sector publisher requires `marine_sensor_msgs`.
If rosdep does not install it automatically on Ubuntu/ROS 2 Humble, install it
directly:

```bash
sudo apt install ros-humble-marine-sensor-msgs
```

Clone the `marine_radar_control_msgs` repo (not included in rosdistro): 

```git clone git@github.com:CCOMJHC/marine_radar_control_msgs.git```


### Optional Packages

Plugin for rqt to view data and control settings: https://github.com/CCOMJHC/rqt_marine_radar

### Build

From the workspace root, build the local control messages and v2 driver:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_v2 build \
  --build-base build_v2 \
  --install-base install_v2 \
  --packages-select marine_radar_control_msgs simrad_halo_driver_v2
source install_v2/setup.bash
```

The separate `build_v2`, `install_v2`, and `log_v2` directories ensure
that building or sourcing this package does not replace an existing
`simrad_halo_radar` installation.


## Usage

Run node with `ros2 run` or `ros2 launch`: 

```ros2 run simrad_halo_driver_v2 simrad_halo_driver_v2``` or

```ros2 launch simrad_halo_driver_v2 simrad_halo_driver_v2.launch.xml```

By default, the driver will scan all available interfaces. To restrict which interface(s) to use, specify the list of IP local addresses using the `hostIPs` parameter.

The radar starts up in stand-by mode and must be switched into transmit mode before it will produce data. 

### Command-Line Mode Change

To do this from the command line, publish a single `RadarControlValue` message to the `change_state` topic for the corresponding frequency (`/halo_a` or `/halo_b`), thusly:

```ros2 topic pub -1 /halo_a/change_state marine_radar_control_msgs/msg/RadarControlValue "{key: 'status', value: 'transmit'}"```

To switch the radar back into stand-by mode, publish another message with the value `standby`, as follows: 

```ros2 topic pub -1 /halo_a/change_state marine_radar_control_msgs/msg/RadarControlValue "{key: 'status', value: 'standby'}"```

### RQT Plugin Mode Change

Use the `rqt_marine_radar` plugin to switch the radar mode between transmit and standby using the GUI interface in rqt.


## Nodes

### simrad_halo_driver_v2 Node 

The `simrad_halo_driver_v2` node publishes data and state from each of the
dual frequencies of the HALO radar. The two channels are addressed by
`/halo_a` and `/halo_b`. `<radar_freq_address>` in the topic names below
refers to either topic prefix.

#### Publishers:

| Published Topic              | Data Type                                         |
|------------------------------|---------------------------------------------------|
| `<radar_freq_address>/data`  | `marine_sensor_msgs::msg::RadarSector`            |
| `<radar_freq_address>/raw_data` | `simrad_halo_driver_v2::msg::HaloRadarSector` |
| `<radar_freq_address>/events` | `simrad_halo_driver_v2::msg::HaloRadarEvent`     |
| `<radar_freq_address>/state` | `marine_radar_control_msgs::msg::RadarControlSet` |

`raw_data` preserves every spoke in wire order, including spokes whose status
is not `0x02`. It publishes the exact sector and spoke headers, packed echo
bytes, raw range and angle fields, scan numbers, kernel UDP arrival timestamp,
arrival gap, source endpoint, packet sizes, and driver-generated revolution
counters. Linux socket receive-queue overflow counters are also included when
the kernel supplies them, which distinguishes local socket overload from loss
upstream of the host. A synthetic message with `message_type=DATA_MISSING` is published
immediately before a sector when the forward raw-angle sequence contains a
gap; `missing_raw_angles` lists every expected raw angle that was absent. The
following received sector repeats this list and describes the transition in
`preceding_continuity` so the evidence survives loss of the synthetic message
during ROS recording.

The driver assumes received UDP sectors remain in capture order, while allowing
gaps between them. In other words, every newly received sector is treated as
later than the previous received sector; an even raw-angle jump is reported as
the explicit set of intervening missing spoke angles. The UDP transport itself
does not prove this ordering, so an externally reordered datagram would be
indistinguishable from forward loss under this policy.

Each spoke also includes the untouched 16-bit heading word and a decoded ego
heading. The lower 12 bits are converted with `raw * 360 / 4096`; bit `0x4000`
marks true north, while a clear bit means magnetic north. `heading_valid` is
false if another upper bit is present. The sector repeats the first valid
heading and sets `heading_consistent` only if all valid spoke headings in that
sector have the same value and north reference.

The expected raw-angle increment of two is the behavior observed for the
Simrad HALO 24 used by this project: the 4096-value angle circle therefore
normally carries 2048 spokes. It is an explicit continuity assumption, not a
claim that every Navico model or operating mode must use that lattice.

The `events` topic reports missing data, ambiguous angle transitions,
malformed packets, kernel receive-queue drops, long arrival gaps, and unusual
spoke statuses. The same events plus a compact record for every received
sector are flushed immediately to a local JSONL audit file. The default
location is
`<package-share>/simrad_halo_driver_v2/logs/<radar_freq_address>/<UTC>_events.jsonl`.
The package share directory is resolved at runtime through the ROS ament index,
so it contains no path from the build PC. Set `event_log_directory` to override
it. The radar serial number recovered during discovery is included
in raw sectors, events, and JSONL records. If the local
`received_packet_sequence` is
consecutive but the bag sequence is not, the loss happened after UDP reception
(DDS or recording); a gap already present in the local file happened earlier.

At every completed revolution the driver also writes a summary to stdout and
as a `record_type="revolution_summary"` JSONL record. It contains received
sector/spoke counts, detected inter-sector and within-packet missing-spoke
counts, malformed sector counts, ambiguous transitions, and non-`0x02` spoke
counts. Each received sector JSONL record also contains
`internal_missing_spoke_count` and `invalid_spoke_count`. These
statistics are deliberately not published in a ROS message. The first
revolution is marked `partial_revolution=true` because capture may start in the
middle of a sweep. The driver cannot know the exact number of absent UDP
datagrams because sector spoke counts vary, so `missing_sector_gap_count` is
the number of contiguous inter-sector gap incidents rather than an inferred
datagram count.


#### Subscriptions:

| Subscribed Topic                    | Data Type                                           | 
|-------------------------------------|-----------------------------------------------------|
| `<radar_freq_address>/change_state` | `marine_radar_control_msgs::msg::RadarControlValue` |


#### Parameters:

| Parameter                                      | Data Type                  | Default Value |
|------------------------------------------------|----------------------------|---------------|
| `hostIPs`                                      | `std::vector<std::string>` | N/A           |
| `<radar_freq_address>.frame_id`                | `std::string`              | `"radar"`     |
| `<radar_freq_address>.range_correction_factor` | `double`                   | `1.024`       |
| `<radar_freq_address>.raw_data_qos_depth`      | `int`                      | `512`         |
| `<radar_freq_address>.radar_model`             | `std::string`              | `"Simrad HALO 24"` |
| `<radar_freq_address>.enable_event_logging`    | `bool`                     | `true`        |
| `<radar_freq_address>.require_event_log`       | `bool`                     | `true`        |
| `<radar_freq_address>.event_qos_depth`         | `int`                      | `256`         |
| `<radar_freq_address>.arrival_gap_warning_ms`  | `double`                   | `100.0`       |
| `<radar_freq_address>.event_log_directory`     | `std::string`              | `<resolved package share>/logs` |

The raw topic uses reliable, volatile, keep-last QoS. Its configurable depth
defaults to 512 messages (approximately eight revolutions when packets contain
32 spokes and a revolution contains 2048 spokes). Avoid unbounded keep-all
history because each normal raw message carries roughly 17 KB of wire data.
The event topic uses the same reliable, volatile, keep-last policy with its
own configurable depth.

With `require_event_log=true`, failure to create or open the JSONL audit file
stops that radar subnode at startup. This is intentional for data collection:
the machine cannot silently continue without the independent loss evidence.
Set it to `false` only when ROS publication without a local audit file is an
acceptable degraded mode.

## Driver self-test

Build with tests enabled (the default) and run:

```bash
colcon --log-base log_v2 test \
  --build-base build_v2 \
  --install-base install_v2 \
  --packages-select simrad_halo_driver_v2
colcon test-result --test-result-base build_v2 --verbose
```

The suite creates HALO-format synthetic datagrams and covers raw field, range,
heading, and intensity decoding; all spoke statuses; exact loss lists
(including wrap and a near-full revolution); a complete 2048-spoke revolution;
malformed and oversized datagrams; counter wrap/reset; and deterministic random
packet stress. It also sends real multicast UDP over the loopback interface and
runs an end-to-end ROS test for raw, legacy, and event topics plus flushed JSONL
logging. The required-log failure path is tested as well. The multicast tests
require a host kernel that permits loopback multicast sockets.


#### Radar State Parameters:

The following parameters are used to control the radar. They can be set either via the command line (as example above) or via the `rqt_marine_radar` plugin. The radar state is published on the `<radar_freq_address>/state` via a `RadarControlSet` message.

| Parameter               | Description                 | Values                                            | 
|-------------------------|-----------------------------|---------------------------------------------------|
| `status`                | Radar state                 | `standby`, `transmit`                             |
| `range`                 | Radar range [meters]        | `25 m` to `75,000 m`                                 |
| `mode`                  | Radar mode                  | `custom`, `harbor`, `offshore`, `weather`, `bird` |
| `gain`                  | Radar gain                  | `0` to `100`                                         |
| `sea_clutter`           | Sea clutter mode            | `0` to`100`                                         |
| `rain_clutter`          | Rain clutter mode           | `0` to `100`                                         |
| `noise_rejection`       | Noise rejection             | `off`, `low`, `medium`, `high`                    |
| `target_expansion`      | Target expansion            | `off`, `low`, `medium`, `high`                    |
| `inteference_rejection` | Interference rejection      | `off`, `low`, `medium`, `high`                    |
| `target_separation`     | Target separation           | `off`, `low`, `medium`, `high`                    |
| `scan_speed`            | Scan speed                  | `off`, `medium`, `high`                           |
| `doppler_mode`          | Doppler mode                | `off`, `normal`, `approaching_only`               |
| `doppler_speed`         | Doppler speed               | `0.5` to `15.95`                                     |
| `antenna_height`        | Antenna height [meters]     | `0.0 m` to `30.175 m`                                |
| `bearing_alignment`     | Bearing alignment [degrees] | `0.0 degrees` to `360.0 degrees`                           |
| `sidelobe_suppression`  | Sidelobe suppression mode   | `0` to `100`                                         |
| `lights`                | Halo lights                 | `off`, `low`, `medium`, `high`                    |


## Troubleshooting

To make sure route is available: 

```sudo route add -net 224.0.0.0 netmask 224.0.0.0 eth0```

Switches and routers between the radar and the machine running the driver may interfere with multicast packets. Consult network equipment documentation or simply the network path.
