# WolvMarine ROS packages and catkin workspace

- `external_packages` : Packages that are not indexed by ROS.
- `wolvmarine_bringup` : Launch files and scripts for bringing up the remote and
  base-station computers.
- `wolvmarine_control` : Control configuration including teleoperation controls,
  PID controllers, and PID interpreter.
- `wolvmarine_drivers` : Launch files for the Velodyne LiDARs and Ladybug
  cameras.
- `wolvmarine_fixer` : Fixes sensory inputs to ROS standard coordinate frames.
- `wolvmarine_localization` : Mapping and GPS.
- `wolvmarine_msgs` : Messages.
- `wolvmarine_navigation` : Path planning and waypoint following.
- `wolvmarine_tasks` : RobotX task definitions and implementations, including
  the heartbeat signal.
- `wolvmarine_tests` :
- `wolvmarine_transform` : Broadcasts the wolvmarine transforms.
- `wolvmarine_viz` : Visualization configuration for rviz.

## Installing dependencies

To install all the dependencies for the wolvmarine package, run the following
in the base directory of the wolvmarine workspace:

    rosdep install --from-paths src --ignore-src -r -y

## Action items

Clean up the `CMakeLists.txt`, `package.xml`, and `setup.py` files. In fact,
everything needs a little cleanup.

Update `README`'s and documentation.

Although generally not advised to make a `utils`-type package, it may be
worthwhile in this application to reduce the number of copy-pasted functions.

In general, launch files should adopt the `machine` tag usage to specify which
computer to run the given node on. If this were done, the remote computer would
not need to be constantly ssh'd into in order to run a node on it.

The `rostest` utility should be adopted to test individual packages. Unit tests
should be written and checked against after any major revision.

We should ensure that subscribers that should be latched are latched. In
addition, there's a lot of opportunity for placing computation into ROS
nodelets.

Following are action items corresponding to each of the individual packages.

- `external_packages`

    - This folder and its included packages should be moved to the base
      directory of the repository so that they aren't rebuilt every time that
      this workspace is changed.

      When moved to the base directory, make sure to include instructions and
      write scripts for `catkin build`ing these external packages and updating
      each user's `.bashrc` and `.zshrc` files as necessary.

    - The `ladybug3_ros` camera package is non-ideal and should be replaced.
      However, the current plan is to replace the camera entirely. Software
      support should be a purchase consideration for the next paneramic camera,
      as support for the current camera is sub-par.

    - The `advanced_navigation_driver` package was developed externally, but it
      would be beneficial to modify this package directly instead of having to
      use wolvmarine_fixer to alter its outputs. Currently, outputs do not
      comply to the ROS standard, specifically NED coordinate frames need to be
      converted to ENU coordinate frames. In addition, all outputs need to be
      timestamped, specifically TwistStamped messages.

- `wolvmarine_bringup`

    - The bringup launch files should be changed to utilize the `machine` tag,
      as well as adhere to the `remote` and `base` taxonomy.

- `wolvmarine_control`

    - Ensure that the *use_lateral_thrusters* in `pid_interpreter.py` is
      working and an appropriate way to handle switching between PID controller
      modes. Specifically, is it better to create a topic for enabling/disabling
      the lateral PID controller itself, rather than doing this at the
      interpreter level?

    - Simplify `pid_interpreter.py`, there's a lot of unnecessary stuff in it.

    - Update `README.md`

    - Cleanup `controls.launch`

- `wolvmarine_drivers`

    - Finish cRIO files and functions.

    - Ensure that everything is working as intended, with the appropriate
      namespacing.

    - Resolve the issues that arose when running multiple instances of
      rosserial. Unable to run multiple instances without putting each into a
      separate namespace. This may be the extent of the fix.

    - Develop a pointcloud nodelet that fuses the two (stern and bow) LiDARs'
      pointclouds together. From this, the nodelet should be able to condense
      all the points in a given region and coordinate frame into a laserscan
      output. Given the tasks being solved for the next competition, it may be
      more benficial to implement an octomap (3D mapping) and compress from
      that point to a 2D view for navigation purposes. That way, the top of a
      *light_signal_buoy* may be more identifiable.

      Joint LiDAR: set min/max degree bounds for each, `laser_assembler` for
      point cloud aggregation, `laser_geometry`
      `transformLaserScanToPointCloud()`. Check whether to use optical
      or inertial coordinate frame.


    - Add the `laser_filter.yaml` parameters directly into the velodyne launch
      files, if still applicable after the above step.

    - Develop (if necessary) the socket communication for the heartbeat to be
      broadcast.

- `wolvmarine_fixer`

    - This package has been a bandaid for quick fixes. The actual fixes should
      be implemented were appropriate.

- `wolvmarine_localization`

    - Finish and verify sensor fusion. Current main sources of odometry are
      the IMU, GPS, and pointcloud-based pose estimation.

    - Look into `laser_scan_matcher` for getting pose estimates from
      pointclouds.

    - Verify mapping.

    - Get AMCL implemented and working.

    - Get the map server and saver working. Possibly move mapping stuff to a
      different package.

- `wolvmarine_navigation`

    - Verify costmap.

    - Finish action framework.

- `wolvmarine_transform`

    - Verify and cleanup transforms after modifying `external_packages`.

    - Consider adopting `robot_state_publisher`.

- `wolvmarine_rviz`

    - Settle on a default configuration.

    - Define a minimal data configuration, for use when sending between the
      remote and base computers in a congested network band.
