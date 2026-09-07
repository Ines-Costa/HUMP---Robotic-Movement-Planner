#!/usr/bin/env python

# see https://github.com/ros/catkin/blob/noetic-devel/doc/user_guide/setup_dot_py.rst#using-packagexml-in-setuppy
# and https://docs.python.org/3/distutils/setupscript.html
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['iiwa_udp_communication'],
    package_dir={'': 'src'}
)

setup(**d)
