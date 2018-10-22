///////////////////////////////////////////////////////////////////////////
// Joan Pau Beltran
//  Modificat per donar suport a les cameres Ladybug3 de Pointgrey.
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin, Ken Tossell
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

// $Id: dev_camera1394.cpp 35607 2011-01-30 01:28:30Z joq $

/** @file

    @brief libdc1394 digital camera library interface implementation
 
    This device interface is partly derived from the Player 1394
    camera driver.

    The ROS image pipeline provides Bayer filtering at a higher level
    (in image_proc).  In some cases it is useful to run the driver
    without the entire image pipeline, so libdc1394 Bayer decoding is
    also provided here.

 */

#include <stdint.h>

#include "yuv.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "dev_ladybug.h"
#include "featuresladybug.h"
#include "modes.h"

#define NUM_DMA_BUFFERS 4

// @todo eliminate these macros
//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[Ladybug::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[Ladybug::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }


using namespace ladybug;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Ladybug::Ladybug():
  camera_(NULL)
{}

Ladybug::~Ladybug() 
{
  SafeCleanup();
}

void Ladybug::findBayerPattern(const char* bayer)
{
  // determine Bayer color encoding pattern
  // (default is different from any color filter provided by DC1394)
  BayerPattern_ = (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM;
  if (0 == strcmp(bayer, "bggr"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_BGGR;
    }
  else if (0 == strcmp(bayer, "grbg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GRBG;
    }
  else if (0 == strcmp(bayer, "rggb"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_RGGB;
    }
  else if (0 == strcmp(bayer, "gbrg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GBRG;
    }
  else if (0 != strcmp(bayer, ""))
    {
      ROS_ERROR("unknown bayer pattern [%s]", bayer);
    }
}

bool Ladybug::findBayerMethod(const char* method)
{
  // Do Bayer conversion in the driver node?
  bool DoBayer = false;                 // return value
  if (0 != strcmp(method, "")
      && BayerPattern_ != DC1394_COLOR_FILTER_NUM)
    {
      DoBayer = true;                   // decoding in driver
      // add method name to message:
      ROS_WARN("[%s] Bayer decoding in the driver is DEPRECATED;"
               " image_proc decoding preferred.", method);

      // Set decoding method
      if (!strcmp(method, "DownSample"))
        BayerMethod_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
      else if (!strcmp(method, "Simple"))
        BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;
      else if (!strcmp(method, "Bilinear"))
        BayerMethod_ = DC1394_BAYER_METHOD_BILINEAR;
      else if (!strcmp(method, "HQ"))
        BayerMethod_ = DC1394_BAYER_METHOD_HQLINEAR;
      else if (!strcmp(method, "VNG"))
        BayerMethod_ = DC1394_BAYER_METHOD_VNG;
      else if (!strcmp(method, "AHD"))
        BayerMethod_ = DC1394_BAYER_METHOD_AHD;
      else
        {
          ROS_ERROR("Unknown Bayer method [%s]. Using ROS image_proc instead.",
                    method);
          DoBayer = false;
        }
    }
  return DoBayer;
}

/** Open the 1394 device and start streaming
 *
 *  @param newconfig new configuration parameters
 *  @return 0 if successful
 *
 *  TODO (if successful):
 *     * update newconfig.guid
 *     * validate newconfig.video_mode
 *     * initialize Features class
 */
int Ladybug::open(ladybug3_ros_pkg::LadybugConfig &newconfig)
{
  //////////////////////////////////////////////////////////////
  // First, look for the camera
  //////////////////////////////////////////////////////////////

  const char *guid = newconfig.guid.c_str();  // C-style GUID for libdc1394
  int err;
  dc1394_t *d;
  dc1394camera_list_t *list;

  // TODO: make error exit paths clean up resources properly
  d = dc1394_new ();
  if (d == NULL)
    {
      CAM_EXCEPT(ladybug::Exception,
                 "Could not initialize dc1394_context.\n"
                 "Make sure /dev/raw1394 exists, you have access permission,\n"
                 "and libraw1394 development package is installed.");
    }

  err = dc1394_camera_enumerate(d, &list);
  if (err != DC1394_SUCCESS)
    {
      CAM_EXCEPT(ladybug::Exception, "Could not get camera list");
      return -1;
    }
  
  if (list->num == 0)
    {
      CAM_EXCEPT(ladybug::Exception, "No cameras found");
      return -1;
    }
  
  char* temp=(char*)malloc(1024*sizeof(char));
  for (unsigned i=0; i < list->num; i++)
    {
      // Create a camera
      camera_ = dc1394_camera_new (d, list->ids[i].guid);
      if (!camera_)
        ROS_WARN_STREAM("Failed to initialize camera with GUID "
                        << std::hex << list->ids[i].guid);
      else
        ROS_INFO_STREAM("Found camera with GUID "
                        << std::hex << list->ids[i].guid);

      uint32_t value[3];
      
      value[0]= camera_->guid & 0xffffffff;
      value[1]= (camera_->guid >>32) & 0x000000ff;
      value[2]= (camera_->guid >>40) & 0xfffff;
      
      sprintf(temp,"%06x%02x%08x", value[2], value[1], value[0]);
      if (strcmp(guid,"")==0)
        {
          ROS_INFO_STREAM("No guid specified, using first camera found, GUID: "
                          << std::hex << camera_->guid);
          device_id_ = std::string(temp);
          break;
        }

      ROS_DEBUG("Comparing %s to %s",guid,temp);
      if (strcmp(temp,guid)==0)
        {
          device_id_=guid;
          break;
        }
      SafeCleanup();
    }
  free (temp);
  dc1394_camera_free_list (list);
  
  if (!camera_)
    {
      if (strcmp(guid,"")==0)
        { 
          CAM_EXCEPT(ladybug::Exception, "Could not find camera");
        }
      else
        {
          CAM_EXCEPT_ARGS(ladybug::Exception,
                          "Could not find camera with guid %s", guid);
        }
      return -1;
    }

  //ROS_INFO_STREAM("camera model: " << camera_->vendor << " " << camera_->model);

  //////////////////////////////////////////////////////////////
  // initialize camera
  //////////////////////////////////////////////////////////////

  // resetting some cameras is not a good idea
  if (newconfig.reset_on_open
      && DC1394_SUCCESS != dc1394_camera_reset(camera_))
    {
      // reset failed: log a warning, but continue
      ROS_WARN("Unable to reset camera (continuing).");
    }

  // first, set parameters that are common between Format7 and other modes
  if (false == Modes::setIsoSpeed(camera_, newconfig.iso_speed))
    {
      SafeCleanup();
      CAM_EXCEPT(ladybug::Exception,
                 "Unable to set ISO speed; is the camera plugged in?");
      return -1;
    }
  
  dc1394operation_mode_t operation_mode;
  if (DC1394_SUCCESS !=  dc1394_video_get_operation_mode(camera_,&operation_mode))
        {
          ROS_ERROR("Could not determine operation mode");
          return false;
        }
  if(operation_mode==DC1394_OPERATION_MODE_LEGACY){
      ROS_INFO_STREAM("Operation mode legacy ");
  }
  else if (operation_mode==DC1394_OPERATION_MODE_1394B){
      ROS_INFO_STREAM("Operation mode B");
  }

  // set video mode
  videoMode_ = Modes::getVideoMode(camera_, newconfig.video_mode);
  if (DC1394_SUCCESS != dc1394_video_set_mode(camera_, videoMode_))
    {
      SafeCleanup();
      CAM_EXCEPT(ladybug::Exception, "Failed to set video mode");
      return -1;
    }

  //////////////////////////////////////////////////////////////
  // special handling for Format7 modes
  //////////////////////////////////////////////////////////////

  findBayerPattern(newconfig.bayer_pattern.c_str());
  DoBayerConversion_ = findBayerMethod(newconfig.bayer_method.c_str());

  if (dc1394_is_video_mode_scalable(videoMode_) == DC1394_TRUE)
    {
      // set Format7 parameters
      if (!format7_.start(camera_, videoMode_, newconfig))
        {
          SafeCleanup();
          CAM_EXCEPT(ladybug::Exception, "Format7 start failed");
          return -1;
        }
    }
  else
    {
         ROS_ERROR_STREAM("Only Format7 supported!");
    }
    
  //////////////////////////////////////////////////////////////
  // get current color coding
  //////////////////////////////////////////////////////////////
  if (DC1394_SUCCESS !=
      dc1394_get_color_coding_from_video_mode(camera_, videoMode_, &colorCoding_) )
    {
      SafeCleanup();
      CAM_EXCEPT(ladybug::Exception, "Format7 start failed");
      return -1;
    }

  //////////////////////////////////////////////////////////////
  // start the device streaming data
  //////////////////////////////////////////////////////////////

  // Set camera to use DMA, improves performance.
  ROS_INFO("Setting DMA");
  if (DC1394_SUCCESS != dc1394_capture_setup(camera_, NUM_DMA_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT))
    {
      SafeCleanup();
      CAM_EXCEPT(ladybug::Exception, "Failed to open device!");
      return -1;
    }
    ROS_INFO("DMA set.");
  
  // Start transmitting camera data
  if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_ON))
    {
      SafeCleanup();
      CAM_EXCEPT(ladybug::Exception, "Failed to start device!");
      return -1;
    }
    ROS_INFO("Camera transmitting!");

  //////////////////////////////////////////////////////////////
  // initialize feature settings
  //////////////////////////////////////////////////////////////

  // TODO: pass newconfig here and eliminate initialize() method
  features_.reset(new Features(camera_));
 
  previous_tstmp=ros::Time::now();
 
  return 0;
}


/** Safe Cleanup -- may get called more than once. */
void Ladybug::SafeCleanup()
{
  if (camera_)
    {
      format7_.stop();
      dc1394_capture_stop(camera_);
      dc1394_camera_free(camera_);
      camera_ = NULL;
    }
}


/** close the 1394 device */
int Ladybug::close()
{
  if (camera_)
    {
      if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_OFF) || DC1394_SUCCESS != dc1394_capture_stop(camera_))
        ROS_WARN("unable to stop camera");
    }

  // Free resources
  SafeCleanup();

  return 0;
}

std::string bayer_string(dc1394color_filter_t pattern, unsigned int bits)
{
  if (bits == 8)
  {
    switch (pattern)
      {
      case DC1394_COLOR_FILTER_RGGB:
        return sensor_msgs::image_encodings::BAYER_RGGB8;
      case DC1394_COLOR_FILTER_GBRG:
        return sensor_msgs::image_encodings::BAYER_GBRG8;
      case DC1394_COLOR_FILTER_GRBG:
        return sensor_msgs::image_encodings::BAYER_GRBG8;
      case DC1394_COLOR_FILTER_BGGR:
        return sensor_msgs::image_encodings::BAYER_BGGR8;
      default:
        return sensor_msgs::image_encodings::MONO8;
      }
  }
  else if (bits == 16)
  {
    // FIXME: should add bayer_XXXX16 modes to sensor_msgs?
    return sensor_msgs::image_encodings::MONO16;
  }

  return sensor_msgs::image_encodings::MONO8;
}

/** Return 6 images frame */
void Ladybug::readMultipleData(
    sensor_msgs::Image& image0,
    sensor_msgs::Image& image1,
    sensor_msgs::Image& image2,
    sensor_msgs::Image& image3,
    sensor_msgs::Image& image4, 
    sensor_msgs::Image& image5,
	std::string publish_monocolour)
{
  ROS_ASSERT_MSG(camera_, "Attempt to read from camera that is not open.");

  dc1394video_frame_t * frame = NULL;
  //Wait for next frame.
  dc1394_capture_dequeue (camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);
  if (!frame)
    {
      CAM_EXCEPT(ladybug::Exception, "Unable to capture frame");
      return;
    }
    
  current_tstmp=ros::Time::now(); 
  elapsed=current_tstmp-previous_tstmp;
  previous_tstmp=current_tstmp;
  
  ROS_INFO_STREAM("Elapsed time btween frames: " << elapsed.toSec() << " sec.");
  
  dc1394video_frame_t frame1 = *frame;

  uint8_t* capture_buffer = reinterpret_cast<uint8_t *>(frame1.image);

  ROS_ASSERT(capture_buffer);   

  image0.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image1.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image2.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image3.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image4.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image5.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );

   if (dc1394_is_video_mode_scalable(videoMode_))
     {

      sensor_msgs::ImagePtr image[6];
      for (int i=0; i<6; i++)
         image[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);          
       format7_.unpackData(image, capture_buffer, publish_monocolour);
       image0=*image[0];
       image1=*image[1];
       image2=*image[2];
       image3=*image[3];
       image4=*image[4];
       image5=*image[5];
       
     }
   else
     {
       CAM_EXCEPT(ladybug::Exception, "Unknown image mode");
       return;
     }
   if (DC1394_SUCCESS != dc1394_capture_enqueue(camera_, frame)){
        ROS_WARN("Unable to enqueue last frame");
    }

}
/** Return 6 compressed images frame */
void Ladybug::readMultipleDataCompressed(
    sensor_msgs::CompressedImage& image0,
    sensor_msgs::CompressedImage& image1,
    sensor_msgs::CompressedImage& image2,
    sensor_msgs::CompressedImage& image3,
    sensor_msgs::CompressedImage& image4, 
    sensor_msgs::CompressedImage& image5,
	std::string publish_monocolour)
{
  ROS_ASSERT_MSG(camera_, "Attempt to read from camera that is not open.");
  dc1394video_frame_t * frame = NULL;
  //Wait for next frame.
  dc1394_capture_dequeue (camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);
  if (!frame)
    {
      CAM_EXCEPT(ladybug::Exception, "Unable to capture frame");
      return;
    }
   
  current_tstmp=ros::Time::now(); 
  elapsed=current_tstmp-previous_tstmp;
  previous_tstmp=current_tstmp;
  
  ROS_INFO_STREAM("Elapsed time btween frames: " << elapsed.toSec() << " sec."); 
    
  dc1394video_frame_t frame1 = *frame;
  uint8_t* capture_buffer = reinterpret_cast<uint8_t *>(frame1.image);

  ROS_ASSERT(capture_buffer);   
  image0.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image1.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image2.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image3.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image4.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  image5.header.stamp = ros::Time( double(frame->timestamp) * 1.e-6 );
  
   if (dc1394_is_video_mode_scalable(videoMode_))
     {
      sensor_msgs::CompressedImagePtr image[6];
      for (int i=0; i<6; i++)
         image[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);          
      format7_.unpackCompressedData(image, capture_buffer, publish_monocolour);
      image0=*image[0];
      image1=*image[1];
      image2=*image[2];
      image3=*image[3];
      image4=*image[4];
      image5=*image[5]; 
     }
   else
     {
       CAM_EXCEPT(ladybug::Exception, "Unknown image mode");
       return;
     }
  //dc1394_capture_enqueue(camera_, frame);
   if (DC1394_SUCCESS != dc1394_capture_enqueue(camera_, frame)){
        ROS_WARN("Unable to enqueue last frame");
    }

 }
