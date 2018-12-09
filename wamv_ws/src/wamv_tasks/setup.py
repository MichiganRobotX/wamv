## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wamv_tasks'],
    package_dir={'': 'src'},
    requires={'rospy', 'sensor_msgs', 'wamv_msgs'}
)
# ,'orocos_kdl'
setup(**setup_args)
