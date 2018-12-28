## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wamv_localization'],
    package_dir={'': 'src'},
    requires={'rospy','geometry_msgs','tf2_ros'}
)
# ,'orocos_kdl'
setup(**setup_args)
