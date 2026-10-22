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

#include "NtripForwarder.h"

#include <mrpt/core/Clock.h>
#include <mrpt/core/format.h>

#include <algorithm>
#include <chrono>
#include <rclcpp/logging.hpp>
#include <vector>

#include "mrpt_sensorlib/mrpt_sensorlib.h"  // short_error_message()

namespace mrpt_sensors
{
namespace
{
// Longest NMEA sentence we accept from the receiver (the standard limit is 82).
constexpr size_t MAX_NMEA_LINE = 200;
}  // namespace

NtripForwarder::NtripForwarder(const Parameters& params, const rclcpp::Logger& logger)
    : params_(params), logger_(logger)
{
}

NtripForwarder::~NtripForwarder()
{
  stop_ = true;
  if (thread_.joinable())
  {
    thread_.join();
  }
  client_.reset();
  if (serial_.isOpen())
  {
    serial_.close();
  }
}

void NtripForwarder::start() { thread_ = std::thread(&NtripForwarder::thread_main, this); }

NtripForwarder::Status NtripForwarder::status() const
{
  std::lock_guard<std::mutex> lck(status_mtx_);
  return status_;
}

void NtripForwarder::set_caster_error(const std::string& err)
{
  std::lock_guard<std::mutex> lck(status_mtx_);
  status_.caster_error = err;
}

void NtripForwarder::set_serial_error(const std::string& err)
{
  std::lock_guard<std::mutex> lck(status_mtx_);
  status_.serial_error = err;
}

void NtripForwarder::thread_main()
{
  RCLCPP_INFO(
      logger_, "[NTRIP] Forwarding %s:%i/%s to serial port '%s'", params_.caster.server.c_str(),
      params_.caster.port, params_.caster.mountpoint.c_str(), params_.serial_port.c_str());

  while (!stop_)
  {
    const double now = mrpt::Clock::nowDouble();
    process_serial(now);
    process_caster(now);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void NtripForwarder::process_serial(double now)
{
  if (!serial_.isOpen())
  {
    if (now < next_serial_open_stamp_)
    {
      return;
    }
    try
    {
      serial_.open(params_.serial_port);
      serial_.setConfig(params_.serial_baud_rate);
      serial_.setTimeouts(0, 0, 0, 0, 1);
      serial_.purgeBuffers();
      gga_line_.clear();
      RCLCPP_INFO(logger_, "[NTRIP] Serial port '%s' open.", params_.serial_port.c_str());
      std::lock_guard<std::mutex> lck(status_mtx_);
      status_.serial_open = true;
      status_.serial_error.clear();
    }
    catch (const std::exception& e)
    {
      if (serial_.isOpen())
      {
        serial_.close();
      }
      next_serial_open_stamp_ = now + params_.reconnect_delay;
      const std::string err =
          "Cannot open serial port '" + params_.serial_port + "': " + short_error_message(e.what());
      if (err != status().serial_error)
      {
        RCLCPP_ERROR(logger_, "[NTRIP] %s", err.c_str());
      }
      set_serial_error(err);
      return;
    }
  }

  // Read the receiver output (the GGA sentences). This also detects USB
  // disconnections, since Read() closes the port then.
  char buf[256];
  size_t n = 0;
  try
  {
    n = serial_.Read(buf, sizeof(buf));
  }
  catch (const std::exception& e)
  {
    serial_.close();
    set_serial_error("Error reading serial port: " + short_error_message(e.what()));
  }

  if (!serial_.isOpen())
  {
    RCLCPP_ERROR(logger_, "[NTRIP] Serial port '%s' lost.", params_.serial_port.c_str());
    next_serial_open_stamp_ = now + params_.reconnect_delay;
    std::lock_guard<std::mutex> lck(status_mtx_);
    status_.serial_open = false;
    if (status_.serial_error.empty())
    {
      status_.serial_error = "Serial port lost";
    }
    return;
  }

  if (n > 0 && params_.send_gga)
  {
    forward_gga(buf, n);
  }
}

void NtripForwarder::forward_gga(const char* data, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    const char c = data[i];
    if (c == '$')
    {
      gga_line_ = c;
    }
    else if (!gga_line_.empty())
    {
      gga_line_ += c;
    }

    if (c == '\n' && gga_line_.size() > 7)
    {
      // "$GPGGA,", "$GNGGA,"...
      if (gga_line_.compare(3, 4, "GGA,") == 0 && client_)
      {
        client_->sendBackToServer(gga_line_);
        std::lock_guard<std::mutex> lck(status_mtx_);
        status_.gga_sent++;
      }
      gga_line_.clear();
    }
    else if (gga_line_.size() > MAX_NMEA_LINE)
    {
      gga_line_.clear();
    }
  }
}

void NtripForwarder::disconnect(const std::string& reason, double now)
{
  RCLCPP_WARN(logger_, "[NTRIP] %s: reconnecting...", reason.c_str());
  client_.reset();
  next_connect_stamp_ = now + params_.reconnect_delay;

  std::lock_guard<std::mutex> lck(status_mtx_);
  status_.connected = false;
  status_.data_rate = 0;
  status_.caster_error = reason;
}

void NtripForwarder::process_caster(double now)
{
  if (!client_)
  {
    if (now < next_connect_stamp_)
    {
      return;
    }

    // A fresh client per attempt, so no state of a failed connection remains.
    // open() blocks for a few seconds at most.
    client_ = std::make_unique<mrpt::hwdrivers::CNTRIPClient>();
    std::string err;
    if (!client_->open(params_.caster, err))
    {
      client_.reset();
      next_connect_stamp_ = mrpt::Clock::nowDouble() + params_.reconnect_delay;
      const std::string msg = "Cannot connect to caster: " + err;
      if (msg != status().caster_error)
      {
        RCLCPP_ERROR(logger_, "[NTRIP] %s", msg.c_str());
      }
      set_caster_error(msg);
      return;
    }

    now = mrpt::Clock::nowDouble();
    rate_window_start_ = now;
    rate_window_bytes_ = 0;
    RCLCPP_INFO(logger_, "[NTRIP] Connected to caster.");

    std::lock_guard<std::mutex> lck(status_mtx_);
    status_.connected = true;
    status_.connected_since = now;
    status_.connections++;
    status_.caster_error.clear();
  }

  std::vector<uint8_t> buf;
  client_->stream_data.readAndClear(buf);

  if (!buf.empty())
  {
    bool forwarded = false;
    if (serial_.isOpen())
    {
      try
      {
        serial_.Write(buf.data(), buf.size());
        forwarded = true;
      }
      catch (const std::exception& e)
      {
        // Reopened by process_serial():
        serial_.close();
        next_serial_open_stamp_ = now + params_.reconnect_delay;
        const std::string err = "Error writing to serial port: " + short_error_message(e.what());
        RCLCPP_ERROR(logger_, "[NTRIP] %s", err.c_str());
        set_serial_error(err);
      }
    }

    rate_window_bytes_ += buf.size();
    std::lock_guard<std::mutex> lck(status_mtx_);
    status_.last_data_stamp = now;
    status_.bytes_received += buf.size();
    if (forwarded)
    {
      status_.bytes_forwarded += buf.size();
    }
    status_.serial_open = serial_.isOpen();
  }
  else
  {
    // A link lost without the TCP connection being closed (e.g. a NAT or WiFi
    // change) would otherwise stall the stream forever:
    const Status st = status();
    const double lastData = std::max(st.last_data_stamp, st.connected_since);
    if (now - lastData > params_.stream_timeout)
    {
      disconnect(mrpt::format("No data from caster for %.1f s", now - lastData), now);
      return;
    }
  }

  if (const double dt = now - rate_window_start_; dt >= 2.0)
  {
    std::lock_guard<std::mutex> lck(status_mtx_);
    status_.data_rate = static_cast<double>(rate_window_bytes_) / dt;
    rate_window_start_ = now;
    rate_window_bytes_ = 0;
  }
}

}  // namespace mrpt_sensors
