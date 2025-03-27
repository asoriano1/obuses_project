#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['moving_only_a6'],
    package_dir={'': 'src'},
    scripts=['scripts/moving_only_a6.py']
)

setup(**d)