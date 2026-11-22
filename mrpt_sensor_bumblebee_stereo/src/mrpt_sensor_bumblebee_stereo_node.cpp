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

// Bumblebee / Bumblebee2 FireWire stereo cameras (libdc1394, through mrpt-hwdrivers).
//
// Topics, relative to the node namespace:
//   left/image_raw, right/image_raw: every frame, only while subscribed.
//   low_rate/left/image_raw, low_rate/right/image_raw: one frame every
//     1/low_rate_hz seconds (e.g. for previews or long recordings).
//   .../camera_info next to each image topic, if calibration_file is given.
// Images are the raw Bayer mosaics sent by the camera (bayer_xxxx8, lossless
// and 3 times smaller than RGB) unless bayer_decoding is true (rgb8).
// Both images of a pair share their stamp: the time the frame was captured.

#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

namespace
{
const char* NODE_NAME = "mrpt_sensor_bumblebee_stereo";
const char* SENSOR_CONFIG = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#
[SENSOR]
driver			= CCameraSensor
process_rate	= ${PROCESS_RATE} // Hz

sensorLabel		= ${SENSOR_LABEL}

grabber_type	= bumblebee_dc1394

preview_decimation = ${CAMERA_PREVIEW_DECIMATION}

pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}
pose_yaw     = ${SENSOR_POSE_YAW}      // (deg)
pose_pitch   = ${SENSOR_POSE_PITCH}
pose_roll    = ${SENSOR_POSE_ROLL}

bumblebee_dc1394_camera_guid   = ${BUMBLEBEE_DC1394_CAMERA_GUID}  // 0 (or not present): the first camera. A hexadecimal number (0x11223344): The GUID of the camera to open
bumblebee_dc1394_camera_unit   = 0  // 0 (or not present): the first camera. 0,1,2,...: The unit number (within the given GUID) of the camera to open (Stereo cameras: 0 or 1)
bumblebee_dc1394_framerate     = ${BUMBLEBEE_DC1394_FRAMERATE} // 1.875, 3.75, 7.5, 15, 30 or 60
bumblebee_dc1394_bayer_decoding = ${BAYER_DECODING}  // false: raw Bayer images (lossless)

capture_grayscale  	= false
)"""";

/** ROS encoding of raw Bayer images, from the camera color filter. */
std::string bayer_encoding(const std::string& pattern)
{
  const std::array<std::string, 4> VALID = {"rggb", "bggr", "gbrg", "grbg"};
  if (std::find(VALID.begin(), VALID.end(), pattern) == VALID.end())
  {
    throw std::invalid_argument("bayer_pattern must be rggb, bggr, gbrg or grbg, not: " + pattern);
  }
  return "bayer_" + pattern + "8";
}

/** Declares a numeric parameter that accepts both integer and double values
 * (e.g. "low_rate_hz:=0" from a launch file is an integer). */
double declare_number(rclcpp::Node& node, const std::string& name, double defaultValue)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  const auto v = node.declare_parameter(name, rclcpp::ParameterValue(defaultValue), descriptor);
  if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return static_cast<double>(v.get<int64_t>());
  }
  return v.get<double>();
}

template <size_t N>
std::array<double, N> read_array(const mrpt::containers::yaml& node, const std::string& key)
{
  const auto entry = node[key];
  const auto& seq = entry.asSequence();
  if (seq.size() != N)
  {
    throw std::invalid_argument("'" + key + "' must have " + std::to_string(N) + " values");
  }
  std::array<double, N> v{};
  for (size_t i = 0; i < N; i++)
  {
    v[i] = seq[i].as<double>();
  }
  return v;
}

/** camera_info of one camera, from a calibration_file entry: image_width,
 * image_height (as in camera_calibration_parsers files), distortion_model and
 * the sensor_msgs/CameraInfo arrays k, d, r, p. */
sensor_msgs::msg::CameraInfo read_camera_info(const mrpt::containers::yaml& node)
{
  sensor_msgs::msg::CameraInfo ci;
  const auto width = node["image_width"].as<int64_t>();
  const auto height = node["image_height"].as<int64_t>();
  if (width <= 0 || height <= 0)
  {
    throw std::invalid_argument("image_width and image_height must be positive");
  }
  ci.width = static_cast<uint32_t>(width);
  ci.height = static_cast<uint32_t>(height);
  ci.distortion_model = node["distortion_model"].as<std::string>();
  if (ci.distortion_model.empty())
  {
    throw std::invalid_argument("distortion_model is empty");
  }
  const auto d = node["d"];
  for (const auto& v : d.asSequence())
  {
    ci.d.push_back(v.as<double>());
  }
  ci.k = read_array<9>(node, "k");
  ci.r = read_array<9>(node, "r");
  ci.p = read_array<12>(node, "p");
  return ci;
}

using ImagePub = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
using InfoPub = rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr;

/** Publishes the left and right images of each stereo frame. */
class StereoPublisher
{
 public:
  explicit StereoPublisher(mrpt_sensors::GenericSensorNode& node) : node_(node)
  {
    low_rate_hz_ = declare_number(node, "low_rate_hz", low_rate_hz_);
    publish_full_rate_ = node.declare_parameter("publish_full_rate", publish_full_rate_);
    raw_encoding_ = bayer_encoding(node.declare_parameter("bayer_pattern", std::string("gbrg")));
    baseline_ = declare_number(node, "stereo_baseline", baseline_);
    const auto calibFile = node.declare_parameter("calibration_file", std::string());
    if (!calibFile.empty())
    {
      load_calibration(calibFile);
    }
  }

  /** Once the node is initialized (it knows the sensor frame). */
  void init()
  {
    frame_rate_ = std::stod(node_.get_parameter("dc1394_framerate").as_string());
    // Reliable, so a recorder gets every image it keeps up with:
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    if (publish_full_rate_)
    {
      full_[0] = node_.create_publisher<sensor_msgs::msg::Image>("left/image_raw", qos);
      full_[1] = node_.create_publisher<sensor_msgs::msg::Image>("right/image_raw", qos);
      if (camera_info_)
      {
        full_info_[0] =
            node_.create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", qos);
        full_info_[1] =
            node_.create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", qos);
      }
    }
    if (low_rate_hz_ > 0)
    {
      low_[0] = node_.create_publisher<sensor_msgs::msg::Image>("low_rate/left/image_raw", qos);
      low_[1] = node_.create_publisher<sensor_msgs::msg::Image>("low_rate/right/image_raw", qos);
      if (camera_info_)
      {
        low_info_[0] =
            node_.create_publisher<sensor_msgs::msg::CameraInfo>("low_rate/left/camera_info", qos);
        low_info_[1] =
            node_.create_publisher<sensor_msgs::msg::CameraInfo>("low_rate/right/camera_info", qos);
      }
    }
    publish_optical_frames();
  }

  void process(const mrpt::obs::CObservation::Ptr& obs)
  {
    const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationStereoImages>(obs);
    ASSERT_(o);

    const double stamp = mrpt::Clock::toDouble(o->timestamp);
    bool lowRate = false;
    if (low_rate_hz_ > 0)
    {
      // Half a camera period of slack, so the rate does not alias to a lower one:
      const double slack = 0.5 / std::max(frame_rate_, 1.0);
      lowRate =
          !last_low_rate_stamp_ || stamp - *last_low_rate_stamp_ >= 1.0 / low_rate_hz_ - slack;
      if (lowRate)
      {
        last_low_rate_stamp_ = stamp;
      }
    }

    const std::array<const mrpt::img::CImage*, 2> images = {
        &o->imageLeft, o->hasImageRight ? &o->imageRight : nullptr};
    for (size_t i = 0; i < images.size(); i++)
    {
      if (!images[i])
      {
        continue;
      }
      const bool full = full_[i] && subscribed(full_[i]);
      const bool low = lowRate && low_[i] && subscribed(low_[i]);
      if (!full && !low)
      {
        continue;
      }
      auto header = node_.create_header(*o);
      header.frame_id = optical_frame(i);
      const auto msg = to_ros(*images[i], header);
      std::optional<sensor_msgs::msg::CameraInfo> info;
      if (camera_info_)
      {
        info = (*camera_info_)[i];
        info->header = header;
      }
      if (full)
      {
        full_[i]->publish(msg);
        if (info)
        {
          full_info_[i]->publish(*info);
        }
      }
      if (low)
      {
        low_[i]->publish(msg);
        if (info)
        {
          low_info_[i]->publish(*info);
        }
      }
    }
  }

 private:
  mrpt_sensors::GenericSensorNode& node_;
  double low_rate_hz_ = 1.0;
  bool publish_full_rate_ = true;
  std::string raw_encoding_;
  double baseline_ = 0.12;  // [m] Bumblebee2
  double frame_rate_ = 15;  // [Hz]
  std::array<ImagePub, 2> full_;
  std::array<ImagePub, 2> low_;
  std::array<InfoPub, 2> full_info_;
  std::array<InfoPub, 2> low_info_;
  // From calibration_file: camera_info (left, right) and the right optical
  // frame pose in the camera body frame.
  std::optional<std::array<sensor_msgs::msg::CameraInfo, 2>> camera_info_;
  std::optional<geometry_msgs::msg::Transform> right_optical_;
  std::optional<double> last_low_rate_stamp_;

  static bool subscribed(const ImagePub& pub)
  {
    return pub->get_subscription_count() + pub->get_intra_process_subscription_count() > 0;
  }

  std::string optical_frame(size_t i) const
  {
    return node_.sensor_frame_id() + (i == 0 ? "_left" : "_right");
  }

  /** calibration_file (YAML): camera_info: {left: {...}, right: {...}} (see
   * read_camera_info()) and, optionally, right_camera: {xyz_m: [x, y, z],
   * quat_xyzw: [x, y, z, w]}: the right optical frame in the camera body frame,
   * which then replaces the nominal stereo_baseline. */
  void load_calibration(const std::string& file)
  {
    std::string where = "camera_info";
    try
    {
      const auto doc = mrpt::containers::yaml::FromFile(file);
      const auto ci = doc["camera_info"];
      std::array<sensor_msgs::msg::CameraInfo, 2> infos;
      for (size_t i = 0; i < 2; i++)
      {
        const std::string side = (i == 0) ? "left" : "right";
        where = "camera_info/" + side;
        infos[i] = read_camera_info(ci[side]);
      }
      camera_info_ = infos;
      if (doc.has("right_camera"))
      {
        where = "right_camera";
        const auto rc = doc["right_camera"];
        const auto t = read_array<3>(rc, "xyz_m");
        const auto q = read_array<4>(rc, "quat_xyzw");
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!(norm > 1e-6))
        {
          throw std::invalid_argument("quat_xyzw has zero norm");
        }
        geometry_msgs::msg::Transform tf;
        tf.translation.x = t[0];
        tf.translation.y = t[1];
        tf.translation.z = t[2];
        tf.rotation.x = q[0] / norm;
        tf.rotation.y = q[1] / norm;
        tf.rotation.z = q[2] / norm;
        tf.rotation.w = q[3] / norm;
        right_optical_ = tf;
      }
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("calibration_file '" + file + "', " + where + ": " + e.what());
    }
    RCLCPP_INFO_STREAM(node_.get_logger(), "Stereo calibration loaded from: " << file);
  }

  /** <sensor_frame_id> is the camera body (x forward, y left, z up) at the
   * left camera; <sensor_frame_id>_left/_right are the optical frames of each
   * camera (z forward, x right, y down). Without a calibrated right camera
   * pose, the right one is at the nominal baseline. */
  void publish_optical_frames()
  {
    for (size_t i = 0; i < 2; i++)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = node_.get_clock()->now();
      tf.header.frame_id = node_.sensor_frame_id();
      tf.child_frame_id = optical_frame(i);
      if (i == 1 && right_optical_)
      {
        tf.transform = *right_optical_;
      }
      else
      {
        tf.transform.translation.y = (i == 0) ? 0.0 : -baseline_;
        tf.transform.rotation.x = -0.5;
        tf.transform.rotation.y = 0.5;
        tf.transform.rotation.z = -0.5;
        tf.transform.rotation.w = 0.5;
      }
      node_.tf_static_bc_->sendTransform(tf);
    }
  }

  sensor_msgs::msg::Image to_ros(
      const mrpt::img::CImage& img, const std_msgs::msg::Header& header) const
  {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.width = static_cast<uint32_t>(img.getWidth());
    msg.height = static_cast<uint32_t>(img.getHeight());
    const uint32_t channels = img.isColor() ? 3 : 1;
    msg.encoding = img.isColor() ? sensor_msgs::image_encodings::RGB8 : raw_encoding_;
    msg.is_bigendian = 0;
    msg.step = msg.width * channels;
    msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
    for (uint32_t row = 0; row < msg.height; row++)
    {
      std::memcpy(
          msg.data.data() + static_cast<size_t>(row) * msg.step,
          img.ptrLine<uint8_t>(static_cast<int32_t>(row)), msg.step);
    }
    return msg;
  }
};

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<mrpt_sensors::GenericSensorNode>(NODE_NAME);
    StereoPublisher stereo(*node);

    node->custom_process_sensor = [&stereo](const mrpt::obs::CObservation::Ptr& o)
    { stereo.process(o); };

    node->init(
        SENSOR_CONFIG,
        {
            {"process_rate", "PROCESS_RATE", "80", false},
            {"sensor_label", "SENSOR_LABEL", "stereo", false},
            {"dc1394_framerate", "BUMBLEBEE_DC1394_FRAMERATE", "15", false},
            // 0: the first camera. A hexadecimal number (0x11223344): the GUID of the camera
            {"dc1394_camera_guid", "BUMBLEBEE_DC1394_CAMERA_GUID", "0", false},
            {"bayer_decoding", "BAYER_DECODING", "false", false},
            {"camera_preview_decimation", "CAMERA_PREVIEW_DECIMATION", "0", false},
            {"sensor_pose_x", "SENSOR_POSE_X", "0", false},
            {"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
            {"sensor_pose_z", "SENSOR_POSE_Z", "0", false},
            {"sensor_pose_yaw", "SENSOR_POSE_YAW", "0", false},
            {"sensor_pose_pitch", "SENSOR_POSE_PITCH", "0", false},
            {"sensor_pose_roll", "SENSOR_POSE_ROLL", "0", false},
        },
        {"SENSOR"});

    stereo.init();
    node->run();

    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(""), "Exception in " << NODE_NAME << " main(): " << e.what());
    return 1;
  }
}
