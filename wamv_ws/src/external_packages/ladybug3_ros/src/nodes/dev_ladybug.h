/* -*- mode: C++ -*- */
///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// NOTE: On 4 Jan. 2011, this file was re-licensed under the GNU LGPL
// with permission of the original GPL authors: Nate Koenig, Andrew
// Howard, Damien Douxchamps and Dan Dennedy.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: dev_camera1394.h 35691 2011-02-02 04:28:58Z joq $

/** @file

    @brief IEEE 1394 digital camera library interface

 */

#ifndef DEV_CAMERA1394_HH
#define DEV_CAMERA1394_HH

#include <dc1394/dc1394.h>

// ROS includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <ladybug3_ros_pkg/LadybugConfig.h>
#include "format7ladybug.h"

class Features;

namespace ladybug
{
  //! Macro for defining an exception with a given parent
  //  (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

  //! A standard Camera1394 exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  class Ladybug
  {
  public:
    Ladybug ();
    ~Ladybug ();

    int open(ladybug3_ros_pkg::LadybugConfig &newconfig);
    int close();
    void readMultipleData (sensor_msgs::Image &image0, sensor_msgs::Image &image1, sensor_msgs::Image &image2, sensor_msgs::Image &image3, sensor_msgs::Image &image4, sensor_msgs::Image &image5, std::string publish_monocolour);
    void readMultipleDataCompressed (sensor_msgs::CompressedImage &image0, sensor_msgs::CompressedImage &image1, sensor_msgs::CompressedImage &image2, sensor_msgs::CompressedImage &image3, sensor_msgs::CompressedImage &image4, sensor_msgs::CompressedImage &image5, std::string publish_monocolour);

    /** check whether CameraInfo matches current video mode
     *
     *  @param image corresponding Image message
     *  @param ci CameraInfo message to check
     *  @return true if camera dimensions match calibration
     */
    bool checkCameraInfo(const sensor_msgs::Image &image,
			 const sensor_msgs::CameraInfo &ci)

    {
      if (format7_.active())
        return format7_.checkCameraInfo(ci);
      else
	return (ci.width == image.width && ci.height == image.height);
    }

    /** set operational parameter fields in CameraInfo message
     *
     *  @param ci CameraInfo message to update
     *
     *  @post CameraInfo fields filled in (if needed):
     *    roi (region of interest)
     *    binning_x, binning_y
     */
    void setOperationalParameters(sensor_msgs::CameraInfo &ci)
    {
      if (format7_.active())
        format7_.setOperationalParameters(ci);
    }

    std::string device_id_;
    boost::shared_ptr<Features> features_;

  private:

    // private data
    dc1394camera_t *camera_;
    dc1394video_mode_t videoMode_;
    dc1394color_coding_t colorCoding_;
    dc1394color_filter_t BayerPattern_;
    dc1394bayer_method_t BayerMethod_;
    bool DoBayerConversion_;
    Format7 format7_;
    ros::Time previous_tstmp;
    ros::Time current_tstmp;
    ros::Duration elapsed;

    void SafeCleanup();
    void findBayerPattern(const char*);
    bool findBayerMethod(const char*);
  };
};

#endif // DEV_CAMERA1394_HH
