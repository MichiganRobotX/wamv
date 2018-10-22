/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
// #include <gazebo_msgs/ModelState.h>
#include <cmath>

#define PI 3.14159265

using namespace gps_common;

static ros::Publisher odom_pub;
// static ros::Publisher gazebo_pub;

std::string frame_id, child_frame_id;
double rot_cov;
float qx, qy, qz, qw;
bool sim_;
double quat[4];

// set origins
bool flag = true;
float ox, oy, prev_x, prev_y;

void quat2eul(double x, double y, double z, double w, double *YPR) {
  YPR[0] = atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
  YPR[1] = asin(-2.0*(x*z - w*y));
  YPR[2] = atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);
}

void eul2quat(double yaw, double pitch, double roll) {
  // Assuming the angles are in radians.
  double c1 = cos(yaw);
  double s1 = sin(yaw);
  double c2 = cos(pitch);
  double s2 = sin(pitch);
  double c3 = cos(roll);
  double s3 = sin(roll);
  double w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
  double w4 = (4.0 * w);
  double x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
  double y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
  double z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;

  quat[0] = x; quat[1] = y; quat[2] = z; quat[3] = w;

}

void imucallback(const sensor_msgs::Imu &msg) {
    qx = msg.orientation.x;
    qy = msg.orientation.y;
    qz = msg.orientation.z;
    qw = msg.orientation.w;
}

void gpscallback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    if(flag) {  // set the origin coordinates here
      ox = easting;
      oy = northing;
      prev_x = 0;
      prev_y = 0;
      flag = false;
    }
    else {  // publish now
      double YPR[3];
      quat2eul(qx,qy,qz,qw,YPR);

      // publish to odom topic
      odom.pose.pose.position.x = easting - ox;
      odom.pose.pose.position.y = northing - oy;
      odom.pose.pose.position.z = 0;

      // calculate yaw
      float dx = (easting - ox - prev_x);
      float dy = (northing -oy - prev_y);
      float yaw = atan2(dy,dx);
      eul2quat(yaw, 0, 0);

      prev_x = easting - ox;
      prev_y = northing - oy;

      /*
      // use the GPS Data
      ROS_WARN("Using GPS Data");
      odom.pose.pose.orientation.x = quat[0];
      odom.pose.pose.orientation.y = quat[1];
      odom.pose.pose.orientation.z = quat[2];
      odom.pose.pose.orientation.w = quat[3];
      */

      // use INS Data
      ROS_WARN("Using INS Data");
      odom.pose.pose.orientation.x = qx;
      odom.pose.pose.orientation.y = qy;
      odom.pose.pose.orientation.z = qz;
      odom.pose.pose.orientation.w = qw;

      // Use ENU covariance to build XYZRPY covariance
      boost::array<double, 36> covariance = {{
        fix->position_covariance[0],
        fix->position_covariance[1],
        fix->position_covariance[2],
        0, 0, 0,
        fix->position_covariance[3],
        fix->position_covariance[4],
        fix->position_covariance[5],
        0, 0, 0,
        fix->position_covariance[6],
        fix->position_covariance[7],
        fix->position_covariance[8],
        0, 0, 0,
        0, 0, 0, rot_cov, 0, 0,
        0, 0, 0, 0, rot_cov, 0,
        0, 0, 0, 0, 0, rot_cov
      }};

      odom.pose.covariance = covariance;
      odom_pub.publish(odom);

      // if (sim_) {
        // publish to gazebo topic
      //   gazebo_msgs::ModelState modelstate;
      //   modelstate.model_name = "kingfisher";
      //   modelstate.pose.position.x = easting - ox;
      //   modelstate.pose.position.y = northing - oy;
      //   modelstate.pose.position.z = 0;
      //
      //   modelstate.pose.orientation.x = 0;
      //   modelstate.pose.orientation.y = 0;
      //   modelstate.pose.orientation.z = qz;
      //   modelstate.pose.orientation.w = qw;
      //   gazebo_pub.publish(modelstate);
      // }
    }
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "wamv_odometry");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99.0);
  priv_node.param<bool>("sim", sim_, false);


  odom_pub = node.advertise<nav_msgs::Odometry>("/gps", 1000000);

  // if(sim_)
  //   gazebo_pub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);

  ros::Subscriber fix_sub = node.subscribe("/navsat", 1000000, gpscallback);
  ros::Subscriber imu_sub = node.subscribe("/imu", 1000000, &imucallback);

  ros::spin();
}
