## General items for test:
- transforms

        the data published into the /tf topic is nothing more than where each frame is in reference to its parent frame in space.

- joint lidar
- PID controller (check inputs)
- status light
- implement network solution
- possible calibration

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

2. build lidar_joint (/sensors/lidar_joint/.../raw) URDF?

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
