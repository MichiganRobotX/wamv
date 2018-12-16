
## Future

- reformat puget and dell

- ladybug3 driver: compressed vs uncompressed images
- lay out network communications: what needs to happen on the puget vs laptop vs transfer. rviz stuff.
- launch file for bagging
- rc solution not utilizing tcp or 2.4/5 GHz bands
- writeup network details
- make additional msgs

- advanced_navigation_driver:
  - change to output TwistStamped msgs
  - change from NED to ENU coordinate frame



## Competition Notes



Namespaces that are good:
  status_light
  motor
  input
  fixer
  transform

... those that need work:
  sensor


Next steps:
  PRIORITY 1:
    drivers/velodyne
    controls
    localization

  THEN:
    mapping
      building map, costmaps
    actions


## Notes

calibrate:
- advanced navigation device

laser_pipeline:
    laser_filter:
    laser_assembler:
laser_scan_matcher: takes pcl and imu to give pose estimate

tf::MessageFilter

- For navigation, it is important that the center of the robot is placed at the rotational center of the robot.
    - imu is rocking back and forth. Should the base_link be paced at the center of floatation?

- For the slam_gmapping node to work properly, you will need to provide 2 transforms:

    - laser -> base_link: Usually a fixed value, broadcast periodically by a robot_state_publisher, or a tf static_transform_publisher.
    - base_link -> odom: Usually provided by the Odometry system.

- Check robot_localization package
    - odom for local planning, mapping, and localization
    - ekf for fuzing gps data
    - "gps data should not be used for local info"

## General items for before shipping:
- setup ladybug3

- joint lidar
    - set min/max degree bounds for each of the lidars
    - laser_assembler for point cloud aggregation
    - laser_geometry transformLaserScanToPointCloud()
    - check optical coordinate frame vs inertial

- lidar camera calibration
    - really just in terms of overlay

- what filters need to be applied? laser scan filters, point cloud filters, image filters?
    - LaserScanBoxFilter

- map server

- status light
- implement network solution

## Status light

1. from heartbeat.

## Other

1. make /odom topic

    - include pose and twist

2. check raw sensor topics

        /sensors/...
            lidar_bow/...
                velodyne_packets (velodyne_msgs/VelodyneScan)
                velodyne_points (sensor_msgs/PointCloud2)
                scan (sensor_msgs/LaserScan)

            lidar_stern/...
                velodyne_packets (velodyne_msgs/VelodyneScan)
                velodyne_points (sensor_msgs/PointCloud2)
                scan (sensor_msgs/LaserScan)

            an_device/...
                NavSatFix (sensor_msgs/NavSatFix)
                Twist (geometry_msgs/Twist)
                Imu (sensor_msgs/Imu)
                SystemStatus
                FilterStatus

            ladybug/...
                camera_0/...
                    imagery
                    ...
                        .../raw
                camera_1/...
                ...

2. build lidar_joint (/sensors/lidar_joint/.../raw)

    - check if there's an upper limit on point_cloud size. e.g., moving from 1000 points to 2000 points
    - build node for joining the two point clouds
    - check on any calibration that needs to be done aside from base_link translations
        -

3. transforms: **measure on wamv** (??? can we get inverse transform without explicit???)

        - lidar_stern ---> lidar_joint
        - lidar_bow ---> lidar_joint

        - lidar_joint ---> base_link (make lidar_joint == base_link)
        - an_device ---> base_link
        - ladybug ---> base_link

        - base_link ---> odom

5. build a map: gmapping

    - gets passed /tf and /lidar_joint/sensor_stream
    - publishes map ---> odom transform and /map (occupancy grid)

6. initialize map_server

    - save map
    - reset map
    - serve map

1. build costmap_2d

    - get_map from map_server and publish costmap

7. amcl (possibly) and path planning

    - local_planner
    - global_planner
