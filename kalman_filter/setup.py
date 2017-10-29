#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['kalman_filter', 'kalman_filter_ros'],
 package_dir={'kalman_filter': 'common/src/kalman_filter', 'kalman_filter_ros': 'ros/src/kalman_filter_ros'}
)

setup(**d)
