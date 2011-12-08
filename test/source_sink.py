#!/usr/bin/env python

import ecto
from ecto_opencv.highgui import imshow, ImageSaver
from image_pipeline.io.source import create_source
from ecto.opts import run_plasm, scheduler_options

import argparse
import sys
import os

parser = argparse.ArgumentParser(description='Saves images from a connect on keystroke.')
parser.add_argument('-o,--output', dest='output', help='The output directory. Default: %(default)s', default='./images')

#ecto options
scheduler_options(parser.add_argument_group('Scheduler Options'))
args = parser.parse_args()

if not os.path.exists(args.output):
    os.makedirs(args.output)

source = create_source(package_name='image_pipeline', source_type='OpenNISource') #optionally pass keyword args here...

depth_saver = ecto.If('depth saver', cell=ImageSaver(filename_format=os.path.join(args.output, 'depth%04d.png')))
image_saver = ecto.If('rgb saver', cell=ImageSaver(filename_format=os.path.join(args.output, 'image%04d.png')))

display = imshow(name='RGB', triggers=dict(save=ord('s')))

plasm = ecto.Plasm()
plasm.connect(source['image'] >> (display['image'], image_saver['image']),
              source['depth'] >> (imshow(name='Depth')['image'], depth_saver['image']),
              display['save'] >> (image_saver['__test__'], depth_saver['__test__'])
              )

run_plasm(args, plasm, locals=vars())
