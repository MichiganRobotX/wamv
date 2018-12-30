## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wolvmarine_controls'],
    package_dir={'': 'src'},
    requires={'geometry_msgs', 'joy', 'nav_msgs', 'pid', 'rospy', 'sensor_msgs',
              'std_msgs', 'wolvmarine_msgs'}
)

setup(**setup_args)
