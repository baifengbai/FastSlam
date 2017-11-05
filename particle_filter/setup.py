#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['particle_filter', 'particle_filter_ros'],
 package_dir={'particle_filter': 'common/src/particle_filter', 'particle_filter_ros': 'ros/src/particle_filter_ros'}
)

setup(**d)
