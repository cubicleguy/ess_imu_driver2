//==============================================================================
//
// 	epson_imu_uart_ros2_node.cpp
//     - ROS2 node for Epson IMU sensor evaluation
//     - This program initializes the Epson IMU and publishes ROS messages in
//       ROS topic /epson_imu as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//     - If the IMU model supports quaternion output
//       then sensor messages are published topic /epson_imu/data with
//       angular_velocity, linear_acceleration, orientation fields updating
//     - If the IMU model does not support quaternion output
//       then sensor messages are published topic /epson_imu/data with only
//       angular_velocity, and linear_acceleration fields updating
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Original Code Development:
//  Copyright (c) 2019, Carnegie Mellon University. All rights reserved.
//
//  Additional Code contributed:
//  Copyright (c) 2019, 2024, Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <termios.h>
#include <iostream>
#include <string>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonUart.h"
#include "sensor_epsonCommon.h"

//=========================================================================
// Timestamp Correction
//
// Time correction makes use of the Epson IMU External Reset Counter function.
// This assumes that the ROS time is accurately sync'ed to GNSS (approx.
// within 100s of microsecs) and the GNSS 1PPS signal is sent to
// Epson IMU's GPIO2_EXT pin. The system latency for calling
// rclcpp::Clock().now() is assumed to be negligible. Otherwise the timestamp
// correction may not be reliable. The get_stamp() method attempts to return
// a timestamp based on the IMU reset count value to exclude time delays
// caused by latencies in the link between the host system and the IMU.
//=========================================================================

class TimeCorrection {
 private:
  const int64_t ONE_SEC_NSEC = 1000000000;
  const int64_t HALF_SEC_NSEC = 500000000;
  // For Gen2 IMUs freq = 46875Hz, max_count = 65535/46875 * 1e9
  const int64_t GEN2_MAX = 1398080000;
  // For Gen3 IMUs freq = 62500Hz, max_count = 65535/62500 * 1e9
  const int64_t GEN3_MAX = 1048560000;
  int64_t max_count;
  int64_t almost_rollover;
  int64_t count_corrected;
  int64_t count_corrected_old;
  int64_t count_old;
  int64_t count_diff;
  int32_t time_sec_current;
  int32_t time_sec_old;
  int64_t time_nsec_current;
  bool rollover;
  bool flag_imu_lead;
  bool is_gen2_imu;

 public:
  TimeCorrection();
  void set_imu(int);
  rclcpp::Time get_stamp(int);
};

// Constructor
TimeCorrection::TimeCorrection() {
  max_count = GEN3_MAX;
  almost_rollover = max_count;
  count_corrected = 0;
  count_old = 0;
  count_diff = 0;
  time_sec_current = 0;
  time_sec_old = 0;
  time_nsec_current = 0;
  count_corrected_old = 0;
  rollover = false;
  flag_imu_lead = false;
  is_gen2_imu = false;
}

//=========================================================================
// TimeCorrection::set_imu
//
// Sets the count thresholds based on external counter reset frequencies
// which may vary depending on the Epson IMU model.
//=========================================================================

void TimeCorrection::set_imu(int epson_model) {
  // max_count depends on IMU model's reset counter freq
  is_gen2_imu = ((epson_model == G320PDG0) || (epson_model == G320PDGN) ||
                 (epson_model == G354PDH0) || (epson_model == G364PDCA) ||
                 (epson_model == G364PDC0));

  max_count = (is_gen2_imu) ? GEN2_MAX : GEN3_MAX;
}

//=========================================================================
// TimeCorrection::get_stamp
//
// Returns the timestamp based on time offset from most recent 1PPS signal.
// Epson IMU has a free-running up-counter that resets on active 1PPS signal.
// Counter value is embedded in the sensor data at the time of sampling.
// Time stamp is corrected based on reset counter retrieved from embedded
// sensor data.
//=========================================================================

rclcpp::Time TimeCorrection::get_stamp(int count) {
  rclcpp::Time now = rclcpp::Clock().now();
  time_sec_current = now.seconds();
  time_nsec_current = now.nanoseconds() % ONE_SEC_NSEC;

  // almost_rollover is arbitrarily set at ~96% of max_count
  almost_rollover = max_count * 0.96;

  count_diff = count - count_old;
  if (count > almost_rollover) {
    rollover = true;
  } else if (count_diff < 0) {
    if (rollover) {
      count_diff = count + (max_count - count_old);
      std::cout << "Warning: time_correction enabled but IMU reset counter "
                   "rollover detected. If 1PPS not connected to IMU GPIO2/EXT "
                   "pin, disable time_correction."
                << std::endl;
    } else {
      count_diff = count;
      count_corrected = 0;
    }
    rollover = false;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  if ((time_sec_current != time_sec_old) && (count_corrected > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current - 1;
  } else if (((count_corrected - count_corrected_old) < 0) &&
             (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
    flag_imu_lead = true;
  } else if (flag_imu_lead && (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
  } else {
    flag_imu_lead = false;
  }
  rclcpp::Time ros_time = rclcpp::Time(time_sec_current, count_corrected);
  time_sec_old = time_sec_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return ros_time;
}

//=========================================================================
// Epson IMU ROS2 C++ Node
//
// 1. Retrieves node parameters from launch file otherwise uses defaults.
// 2. Creates a publisher for IMU messages
// 3. Sets up communication and initializes IMU with launch file settings
// 4. Starts an internal timer to periodically check for incoming IMU data
// 5. Formats and publishes IMU messages until <CTRL-C>
//=========================================================================

using namespace std;
using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
 public:
  explicit ImuNode(const rclcpp::NodeOptions& op) : Node("epson_node", op) {
    ParseParams();
    Init();

    // publisher
    imu_data_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_.c_str(), 10);
    imu_tempc_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      temperature_topic_.c_str(), 20);

    // poll_rate_ must be at least 4000Hz (2x the highest IMU
    // output rate of 2000Hz)
    std::chrono::milliseconds ms((int)(1000.0 / poll_rate_));
    timer_ = this->create_wall_timer(ms, std::bind(&ImuNode::Spin, this));
  }

  ~ImuNode() {
    sensorStop();
    uartRelease();
    gpioRelease();
    seRelease();
    RCLCPP_INFO(this->get_logger(), "Cleanup and shutdown completed.");
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_tempc_pub_;

  char prod_id_[9];  // String to store Device Product ID
  char ser_id_[9];   // String to store Device Serial ID

  // IMU properties
  struct EpsonProperties epson_sensor_;

  // IMU configuration settings
  struct EpsonOptions options_ = {};

  // Buffer for scaled values of IMU read burst
  struct EpsonData epson_data_ = {};

  std::string port_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string temperature_topic_;
  double poll_rate_;

  // Flag for enable/disable time_correction function
  // Time correction requires 1PPS connection to IMU GPIO2_EXT pin
  // and cannot be used with ext_trigger at the same time
  bool time_correction_;

  // Time correction object
  TimeCorrection tc;

  void ParseParams() {
    // NOTE: IMU settings are parsed when this node is launched using .py
    //       launch file in launch/ folder
    //       To change the IMU settings, it is recommended to modify
    //       .py launch file instead of this source file

    std::string key;
    // Defaults for IMU which can be changed by .py launch file
    port_ = "/dev/ttyUSB0";
    frame_id_ = "imu_link";
    imu_topic_ = "/epson_imu/data_raw";
    temperature_topic_ = "/epson_imu/tempc";
    poll_rate_ = 4000.0;
    time_correction_ = false;

    // ext_trigger function should not be enabled when time correction enabled
    bool ext_trigger_en_ = false;

    bool output_32bit_ = false;

    // ext_sel 0 = Sample Counter 1=Reset Counter 2=External Trigger
    options_.ext_sel = 1;

    // drdy_pol 1 = active HIGH 0=active LOW
    options_.drdy_on = true;
    options_.drdy_pol = 1;

    options_.dout_rate = CMD_RATE125;
    options_.filter_sel = CMD_FLTAP32;
    options_.flag_out = true;

    // temp_out must be set true to publish temperature data
    options_.temp_out = true;

    options_.gyro_out = true;
    options_.accel_out = true;
    options_.qtn_out = false;

    // count_out must be set true for time_correction function
    options_.count_out = true;

    options_.checksum_out = true;

    // atti_mode is only valid for attitude/quaternion output
    // atti_mode 1=Euler mode 0=Inclination mode
    options_.atti_mode = 1;

    // atti_profile only is valid for attitude/quaternion output
    // 0=General 1=Vehicle 2=Construction Machinery
    options_.atti_profile = 0;

    // Read parameters
    key = "serial_port";
    if (this->get_parameter(key, port_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(), port_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  port_.c_str());
    }

    key = "frame_id";
    if (this->get_parameter(key, frame_id_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t\t%s", key.c_str(),
                  frame_id_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  frame_id_.c_str());
    }

    key = "time_correction_en";
    if (this->get_parameter(key, time_correction_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), time_correction_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  time_correction_);
    }

    key = "burst_polling_rate";
    if (this->get_parameter(key, poll_rate_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%.1f", key.c_str(), poll_rate_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%.1f",
                  key.c_str(), poll_rate_);
    }

    key = "temperature_topic";
    if (this->get_parameter(key, temperature_topic_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(),
                  temperature_topic_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  temperature_topic_.c_str());
    }

    key = "ext_trigger_en";
    if (this->get_parameter(key, ext_trigger_en_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), ext_trigger_en_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  ext_trigger_en_);
    }

    key = "imu_dout_rate";
    if (this->get_parameter(key, options_.dout_rate)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%d", key.c_str(),
                  options_.dout_rate);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.dout_rate);
    }

    key = "imu_filter_sel";
    if (this->get_parameter(key, options_.filter_sel)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%d", key.c_str(),
                  options_.filter_sel);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.filter_sel);
    }

    key = "quaternion_output_en";
    if (this->get_parameter(key, options_.qtn_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.qtn_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.qtn_out);
    }

    key = "output_32bit_en";
    if (this->get_parameter(key, output_32bit_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%d", key.c_str(), output_32bit_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  output_32bit_);
    }
    if (output_32bit_) {
      options_.temp_bit = true;
      options_.gyro_bit = true;
      options_.accel_bit = true;
      options_.qtn_bit = true;
    }

    key = "atti_profile";
    if (this->get_parameter(key, options_.atti_profile)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%d", key.c_str(),
                  options_.atti_profile);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.atti_profile);
    }

    // Send warning if both time_correction & ext_trigger are enabled
    // Disable ext_trigger if both are enabled
    if (time_correction_ && ext_trigger_en_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Time correction & ext_trigger both enabled");
      RCLCPP_WARN(this->get_logger(),
                  "Ext_trigger will be disabled. 1PPS should be connected to "
                  "GPIO2/EXT");
      options_.ext_sel = 1;
    } else if (!time_correction_ && ext_trigger_en_) {
      options_.ext_sel = 2;
      RCLCPP_INFO(this->get_logger(),
                  "Trigger should be connected to GPIO2/EXT");
    } else if (time_correction_ && !ext_trigger_en_) {
      options_.ext_sel = 1;
      RCLCPP_INFO(
        this->get_logger(),
        "Time correction enabled. 1PPS should be connected to GPIO2/EXT");
    }
  }

  void Init() {
    std::chrono::seconds sec(1);
    rclcpp::Rate one_sec(sec);

    while (rclcpp::ok() && !seInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the Seiko Epson HCL "
                  "layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !gpioInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the GPIO layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !(uartInit(port_.c_str(), BAUD_460800))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Retry to initialize the UART interface: %s in 1 second period...",
        port_.c_str());
      one_sec.sleep();
    }

    while (rclcpp::ok() && !InitImu(&epson_sensor_, &options_)) {
      RCLCPP_WARN(this->get_logger(), "Retry to initialize the IMU...");
      one_sec.sleep();
    }

    // Initialize time correction thresholds if enabled
    if (time_correction_) {
      tc.set_imu(epson_sensor_.model);
    }

    // if quaternion output is enabled set topic to /epson_imu/data
    // Otherwise set topic to /epson_imu/data_raw
    imu_topic_ = (static_cast<bool>(options_.qtn_out) == false)
                   ? "/epson_imu/data_raw"
                   : "/epson_imu/data";

    sensorStart();
    RCLCPP_INFO(this->get_logger(), "Sensor started...");
  }

  bool InitImu(struct EpsonProperties* ptr_epson_sensor_,
               struct EpsonOptions* ptr_options_) {
    RCLCPP_INFO(this->get_logger(), "Checking sensor power on status...");
    if (!sensorPowerOn()) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: failed to power on Sensor. Exiting...");
      return false;
    }

    // Auto-detect Epson Sensor Model Properties
    RCLCPP_INFO(this->get_logger(), "Detecting sensor model...");
    if (!sensorGetDeviceModel(ptr_epson_sensor_, prod_id_, ser_id_)) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: failed to detect sensor model. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Sensor...");
    if (!sensorInitOptions(ptr_epson_sensor_, ptr_options_)) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: could not initialize Epson Sensor. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Epson IMU initialized.");
    return true;
  }

  void PubImuData() {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    auto tempc_msg = std::make_shared<sensor_msgs::msg::Temperature>();

    for (int i = 0; i < 9; i++) {
      imu_msg->orientation_covariance[i] = 0;
      imu_msg->angular_velocity_covariance[i] = 0;
      imu_msg->linear_acceleration_covariance[i] = 0;
    }
    imu_msg->orientation_covariance[0] = -1;
    imu_msg->header.frame_id = frame_id_;

    while (rclcpp::ok()) {
      // Call to read and post process IMU sensor burst data
      // Will return 0 if data incomplete or checksum error
      if (sensorDataReadBurstNOptions(&epson_sensor_, &options_,
                                      &epson_data_)) {
        if (!time_correction_) {
          imu_msg->header.stamp = this->now();
        } else {
          imu_msg->header.stamp = tc.get_stamp(epson_data_.count);
        }

        // Linear acceleration
        imu_msg->linear_acceleration.x = epson_data_.accel_x;
        imu_msg->linear_acceleration.y = epson_data_.accel_y;
        imu_msg->linear_acceleration.z = epson_data_.accel_z;

        // Angular velocity
        imu_msg->angular_velocity.x = epson_data_.gyro_x;
        imu_msg->angular_velocity.y = epson_data_.gyro_y;
        imu_msg->angular_velocity.z = epson_data_.gyro_z;

        // Orientation
        imu_msg->orientation.x = epson_data_.qtn1;
        imu_msg->orientation.y = epson_data_.qtn2;
        imu_msg->orientation.z = epson_data_.qtn3;
        imu_msg->orientation.w = epson_data_.qtn0;

        imu_data_pub_->publish(*imu_msg);

        // Temperature
        tempc_msg->header = imu_msg->header;
        tempc_msg->temperature = epson_data_.temperature;
        tempc_msg->variance = 0;
        imu_tempc_pub_->publish(*tempc_msg);

      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Warning: Checksum error or incorrect delimiter bytes in imu_msg "
          "detected");
      }
    }
  }

  void Spin() { PubImuData(); }
};  // end of class

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer, which ensures a sync of all console
  // output even from a launch file.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto imu_node = std::make_shared<ImuNode>(options);

  rclcpp::spin(imu_node);
  rclcpp::shutdown();
  return 0;
}
