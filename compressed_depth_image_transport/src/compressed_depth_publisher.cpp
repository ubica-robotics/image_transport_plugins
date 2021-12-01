/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "compressed_depth_image_transport/compressed_depth_publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"

#include <rclcpp/parameter_client.hpp>

#include <vector>
#include <sstream>

constexpr int kDefaultPngLevel = 9;
constexpr double kDefaultDepthMax = 10.0;
constexpr double KDefaultDepthQuantization = 100.0;

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_depth_image_transport
{

  void CompressedDepthPublisher::advertiseImpl(
    rclcpp::Node * node,
    const std::string& base_topic,
    rmw_qos_profile_t custom_qos)
  {
    typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
    Base::advertiseImpl(node, base_topic, custom_qos);

    uint ns_len = node->get_effective_namespace().length();
    std::string param_base_name = base_topic.substr(ns_len);
    std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
    param_base_name += ".compressedDepth";

    std::string png_level_param_name = param_base_name + ".png_level";
    rcl_interfaces::msg::ParameterDescriptor png_level_description;
    png_level_description.name = "png_level";
    png_level_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    png_level_description.description = "Compression level for PNG format";
    png_level_description.read_only = false;
    rcl_interfaces::msg::IntegerRange png_range;
    png_range.from_value = 0;
    png_range.to_value = 9;
    png_range.step = 1;
    png_level_description.integer_range.push_back(png_range);
    try {
      config_.png_level = node->declare_parameter(
        png_level_param_name, kDefaultPngLevel, png_level_description);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", png_level_param_name.c_str());
      config_.png_level = node->get_parameter(png_level_param_name).get_value<int64_t>();
    }

    std::string depth_max_param_name = param_base_name + ".depth_max";
    rcl_interfaces::msg::ParameterDescriptor depth_max_description;
    depth_max_description.name = "depth_max";
    depth_max_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    depth_max_description.description = "Maximum depth value (meter)";
    depth_max_description.read_only = false;
    rcl_interfaces::msg::FloatingPointRange depth_max_range;
    depth_max_range.from_value = 0.0;
    depth_max_range.to_value = 100.0;
    depth_max_range.step = 0.01;
    depth_max_description.floating_point_range.push_back(depth_max_range);
    try {
      config_.depth_max = node->declare_parameter(
        depth_max_param_name, kDefaultDepthMax, depth_max_description);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", depth_max_param_name.c_str());
      config_.depth_max = node->get_parameter(depth_max_param_name).get_value<double>();
    }

    std::string depth_quantization_param_name = param_base_name + ".depth_quantization";
    rcl_interfaces::msg::ParameterDescriptor depth_quantization_description;
    depth_quantization_description.name = "depth_quantization";
    depth_quantization_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    depth_quantization_description.description = "Maximum depth value (meter)";
    depth_quantization_description.read_only = false;
    rcl_interfaces::msg::FloatingPointRange depth_quantization_range;
    depth_quantization_range.from_value = 0.0;
    depth_quantization_range.to_value = 150.0;
    depth_quantization_range.step = 0.01;
    depth_quantization_description.floating_point_range.push_back(depth_quantization_range);
    try {
      config_.depth_quantization = node->declare_parameter(
        depth_quantization_param_name, KDefaultDepthQuantization, depth_quantization_description);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", depth_quantization_param_name.c_str());
      config_.depth_quantization = node->get_parameter(depth_quantization_param_name).get_value<double>();
    }

    //node->get_parameter_or<int>("png_level", config_.png_level, kDefaultPngLevel);
    //node->get_parameter_or<double>("depth_max", config_.depth_max, kDefaultDepthMax);
    //node->get_parameter_or<double>("depth_quantization", config_.depth_max, KDefaultDepthQuantization);
  }

  void CompressedDepthPublisher::publish(
    const sensor_msgs::msg::Image& message,
    const PublishFn& publish_fn) const
  {
    sensor_msgs::msg::CompressedImage::SharedPtr compressed_image =
            encodeCompressedDepthImage(message,
                                       config_.depth_max,
                                       config_.depth_quantization,
                                       config_.png_level);
    if (compressed_image)
    {
      publish_fn(*compressed_image);
    }
  }

} //namespace compressed_depth_image_transport

