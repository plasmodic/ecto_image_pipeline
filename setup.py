#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()


d['packages'] = ['ecto_image_pipeline', 'ecto_image_pipeline.io', 'ecto_image_pipeline.io.source']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
