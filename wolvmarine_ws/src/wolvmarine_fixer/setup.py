## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wolvmarine_fixer'],
    package_dir={'': 'src'},
    requires={'geometry_msgs', 'message_filters', 'orocos_kdl', 'rospy',
              'sensor_msgs', 'tf2_ros'}
)

setup(**setup_args)
