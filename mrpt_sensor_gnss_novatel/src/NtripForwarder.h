/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   |                                                                        |
   | Redistribution and use in source and binary forms, with or without     |
   | modification, are permitted provided that the following conditions     |
   | are met:                                                               |
   |                                                                        |
   | 1. Redistributions of source code must retain the above copyright      |
   |    notice, this list of conditions and the following disclaimer.       |
   |                                                                        |
   | 2. Redistributions in binary form must reproduce the above copyright   |
   |    notice, this list of conditions and the following disclaimer in the |
   |    documentation and/or other materials provided with the distribution.|
   |                                                                        |
   | 3. Neither the name of the copyright holder nor the names of its       |
   |    contributors may be used to endorse or promote products derived     |
   |    from this software without specific prior written permission.       |
   |                                                                        |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    |
   | "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      |
   | LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  |
   | A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   |
   | HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |
   | SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       |
   | LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  |
   | DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  |
   | THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    |
   | (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  |
   | OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   |
   |                                                                        |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   | SPDX-License-Identifier: BSD-3-Clause                                  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/comms/CSerialPort.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>

namespace mrpt_sensors
{
/** Forwards the RTCM corrections of an NTRIP caster to the serial port of a
 * GNSS receiver (and the receiver GGA sentences back to the caster), in its
 * own thread.
 *
 * Unlike mrpt::hwdrivers::CNTRIPEmitter, it recovers by itself from:
 * - the caster not being reachable when starting,
 * - silent connection losses (no data for `stream_timeout` seconds),
 * - the serial port disappearing (e.g. USB disconnections).
 *
 * Its state is exposed through status(), for diagnostics.
 */
class NtripForwarder
{
 public:
  struct Parameters
  {
    mrpt::hwdrivers::CNTRIPClient::NTRIPArgs caster;
    std::string serial_port;
    int serial_baud_rate = 9600;
    double stream_timeout = 10.0;  //!< [s] Reconnect if no data arrives for this long
    double reconnect_delay = 5.0;  //!< [s] Wait between connection attempts
    bool send_gga = true;  //!< Send the receiver GGA sentences to the caster
  };

  struct Status
  {
    bool connected = false;  //!< Connected to the caster
    bool serial_open = false;
    double connected_since = 0;  //!< [s, wall clock] valid if connected
    double last_data_stamp = 0;  //!< [s, wall clock] 0 if never received
    double data_rate = 0;  //!< [bytes/s] last few seconds
    uint64_t bytes_received = 0;
    uint64_t bytes_forwarded = 0;  //!< written to the serial port
    uint64_t gga_sent = 0;  //!< GGA sentences sent to the caster
    uint64_t connections = 0;  //!< successful connections so far
    std::string caster_error;  //!< Last caster error, cleared when connected
    std::string serial_error;  //!< Last serial port error, cleared when open
  };

  NtripForwarder(const Parameters& params, const rclcpp::Logger& logger);
  ~NtripForwarder();

  NtripForwarder(const NtripForwarder&) = delete;
  NtripForwarder& operator=(const NtripForwarder&) = delete;

  /// Launches the working thread.
  void start();

  Status status() const;

  const Parameters& parameters() const { return params_; }

 private:
  Parameters params_;
  rclcpp::Logger logger_;

  std::thread thread_;
  std::atomic_bool stop_{false};

  mutable std::mutex status_mtx_;
  Status status_;

  // Only accessed from the working thread:
  std::unique_ptr<mrpt::hwdrivers::CNTRIPClient> client_;
  mrpt::comms::CSerialPort serial_;
  std::string gga_line_;
  double next_connect_stamp_ = 0;
  double next_serial_open_stamp_ = 0;
  double rate_window_start_ = 0;
  uint64_t rate_window_bytes_ = 0;

  void thread_main();
  void process_serial(double now);
  void process_caster(double now);
  void forward_gga(const char* data, size_t len);
  void disconnect(const std::string& reason, double now);
  void set_caster_error(const std::string& err);
  void set_serial_error(const std::string& err);
};

}  // namespace mrpt_sensors
