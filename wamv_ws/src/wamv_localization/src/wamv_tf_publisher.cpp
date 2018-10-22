/*
This node will be broadcasting the transforms for different frames on the wamv,
namely:
(i)     base_link   - the centroid of the robot
(ii)    gps         - the position of the gps wrt the base_link
(iii)   velodyne    - the position of the velodyne wrt the base_link
(iv)    camera      - the position of the camera wrt the base_link
(v)     imu         - the position of the imu wrt the base_link
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

float qx;
float qy;
void odom_callback(const nav_msgs::Odometry &msg){
    qx = msg.pose.pose.position.x;
    qy = msg.pose.pose.position.y;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wamv_tf_publisher");
    ros::NodeHandle nh;

    nh.setParam("use_sim_time", true);

    ros::Subscriber odom_sub = nh.subscribe("/gps/odom", 1000, &odom_callback);

    tf::TransformBroadcaster br;
    tf::Transform tf_velo_base;
    tf::Transform tf_map_base;
    tf::Transform tf_gps_base;
    tf::Transform tf_base_gps;


    ros::Rate rate(10);

    while(nh.ok()){
    	ROS_INFO("Publishing Transform Frames...");
        tf_map_base.setOrigin(tf::Vector3(qx, qy, 0.5));
        tf_map_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(tf_map_base, ros::Time::now(), "map", "base_link"));

        tf_velo_base.setOrigin(tf::Vector3(2.4, 0.0, 1.8));
        tf_velo_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(tf_velo_base, ros::Time::now(), "velodyne", "base_link" ));

        tf_base_gps.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf_base_gps.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(tf_base_gps, ros::Time::now(), "base_link", "gps"));

        tf_gps_base.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf_gps_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(tf_gps_base, ros::Time::now(), "gps", "base_link"));

        // tf_map_base.setOrigin(tf::Vector3(qx, qy, 0.5));
        // tf_map_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        // br.sendTransform(tf::StampedTransform(tf_map_base, ros::Time::now(), "base_link", "map"));

        // tf_velo_base.setOrigin(tf::Vector3(2.4, 0.0, 1.8));
        // tf_velo_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        // br.sendTransform(tf::StampedTransform(tf_velo_base, ros::Time::now(), "base_link", "velodyne" ));

        // tf_base_gps.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // tf_base_gps.setRotation(tf::Quaternion(0, 0, 0, 1));
        // br.sendTransform(tf::StampedTransform(tf_base_gps, ros::Time::now(), "base_link", "gps"));

        // tf_gps_base.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // tf_gps_base.setRotation(tf::Quaternion(0, 0, 0, 1));
        // br.sendTransform(tf::StampedTransform(tf_gps_base, ros::Time::now(), "gps", "base_link"));

        rate.sleep();

    }
    return 0;
}
