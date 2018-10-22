#ifndef DRIVER_JPEG_DISK_H
#define DRIVER_JPEG_DISK_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <signal.h>
#include <math.h>
#include <boost/thread.hpp>

// Includes opencv
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Includes libdc1394
#include <dc1394/dc1394.h>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
using std::string;

class DriverJpeg
{
public:
  //! Constructor.
  DriverJpeg(int argc, char **argv);

  //! Destructor.
  ~DriverJpeg();

  int run();

  //! Publish the message.
  void set_preview_flag(const ros::TimerEvent& event);
  void save_images();
  
  char * directory_;
  float previewTime_;

  ros::NodeHandle nh_;
  ros::Publisher image_laterals_pub_;
  ros::Publisher image_bottom_pub_;
  ros::Timer timer_;
  
  std::vector<unsigned char *> buffer_;
  
  boost::thread saveImagesThread_;
  boost::mutex mtx_;
  bool preview_;
  double time_;
  int nframes_;
  double total_diff_;
  double max_diff_ladybug_;
  int total_lost_frames_;
  int buffer_full_;
  double time_start_;

};

#endif // DRIVER_JPEG_DISK_H
