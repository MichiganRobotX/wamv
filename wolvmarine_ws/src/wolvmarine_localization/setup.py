## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wamv_localization'],
    package_dir={'': 'src'},
    requires={'geometry_msgs', 'hector_compressed_map_transport',
              'hector_geotiff', 'hector_mapping', 'hector_trajectory_server',
              'laser_scan_matcher', 'message_filters', 'nav_msgs', 'orocos_kdl',
              'robot_localization', 'rospy', 'sensor_msgs', 'std_msgs',
              'tf2_geometry_msgs', 'tf2_ros'}
)

setup(**setup_args)
