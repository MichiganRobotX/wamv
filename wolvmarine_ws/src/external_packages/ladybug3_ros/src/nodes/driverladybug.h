/* -*- mode: C++ -*- */
/* $Id: driver1394.h 35611 2011-01-30 05:49:18Z joq $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <camera_info_manager/camera_info_manager.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "dev_ladybug.h"
#include "ladybug3_ros_pkg/LadybugConfig.h"
typedef ladybug3_ros_pkg::LadybugConfig Config;

/** @file

    @brief ROS driver interface for IIDC-compatible IEEE 1394 digital cameras.

*/

namespace ladybug_driver
{

class LadybugDriver
{
public:

  // public methods
  LadybugDriver(ros::NodeHandle priv_nh,
                   ros::NodeHandle camera_nh);
  ~LadybugDriver();
  void poll(void);
  void setup(void);
  void shutdown(void);

private:

  enum CameraSelector {CAMERA0=0,CAMERA1=1,CAMERA2=2,CAMERA3=3,CAMERA4=4,CAMERA5=5};
  static const int NUM_CAMERAS = 6;
  static const std::string CameraSelectorString[NUM_CAMERAS]; //

  // private methods
  void closeCamera();
  bool openCamera(Config &newconfig);
  void publish(const sensor_msgs::ImagePtr image[NUM_CAMERAS]);
  void publishCompressed(const sensor_msgs::CompressedImagePtr image[NUM_CAMERAS]);
  bool read(sensor_msgs::ImagePtr image[NUM_CAMERAS]);
  bool readCompressed(sensor_msgs::CompressedImagePtr image[NUM_CAMERAS]);
  void reconfig(ladybug3_ros_pkg::LadybugConfig &newconfig, uint32_t level);

  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;        // true when reconfig() running

  ros::NodeHandle priv_nh_;             // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  ros::NodeHandle single_camera_nh_[NUM_CAMERAS]; // left/right camera name space handle
  std::string camera_name_;             // camera name

  ros::Publisher image_compressed_pub_[NUM_CAMERAS];
  ros::Publisher camera_info_pub_[NUM_CAMERAS];
  bool compressed_mode_;
  bool publish_compressed_;
  bool separated_channels_;
  std::string publish_monocolour_;

  /** libdc1394 camera device interface */
  boost::shared_ptr<ladybug::Ladybug> dev_;

  /** dynamic parameter configuration */
  ladybug3_ros_pkg::LadybugConfig config_;
  dynamic_reconfigure::Server<ladybug3_ros_pkg::LadybugConfig> srv_;
  ros::Rate cycle_;                     // polling rate when closed

  /** camera calibration information */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_[NUM_CAMERAS];
  bool calibration_matches_[NUM_CAMERAS];            // CameraInfo matches video mode

  /** image transport interfaces */
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_[NUM_CAMERAS];

}; // end class LadybugDriver

}; // end namespace ladybug_driver
