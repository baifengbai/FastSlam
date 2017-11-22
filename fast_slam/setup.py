#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['fast_slam', 'fast_slam_ros'],
 package_dir={'fast_slam': 'common/src/fast_slam', 'fast_slam_ros': 'ros/src/fast_slam_ros'}
)

setup(**d)
