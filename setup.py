#!/usr/bin/env python
from distutils.core import setup

setup(name='Ecto image pipeline',
      version='1.0.0',
      description='Utilities for using the image pipilend with ecto',
      packages=['ecto_image_pipeline', 'ecto_image_pipeline.io', 'ecto_image_pipeline.io.source'],
      package_dir={'':'python'}
)
