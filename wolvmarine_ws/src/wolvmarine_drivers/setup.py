## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wolvmarine_drivers'],
    package_dir={'': 'src'},
    requires={'advanced_navigation_driver', 'laser_filters', 'rospy',
              'rosserial_python', 'rostest', 'sensor_msgs', 'std_msgs',
              'velodyne_driver', 'velodyne_pointcloud'}
)

setup(**setup_args)
