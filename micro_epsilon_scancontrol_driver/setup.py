#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['micro_epsilon_scancontrol_driver', 'micro_epsilon_scancontrol_driver.plugins'],
    package_dir={'': 'src'},
    scripts=['scripts/me_scancontrol_plugins']
)

setup(**d)