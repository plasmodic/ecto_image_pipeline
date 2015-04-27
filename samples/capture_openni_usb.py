#!/usr/bin/env python

import ecto
from ecto_opencv.highgui import imshow, ImageSaver, VideoCapture
from ecto_image_pipeline.io.source import create_source
from ecto.opts import run_plasm, scheduler_options

import argparse
import sys
import os
import time

parser = argparse.ArgumentParser(description='Saves images from a connect on keystroke.')
parser.add_argument('-o,--output', dest='output', help='The output directory. Default: %(default)s', default='./images/%s'%(time.strftime('%Y-%b-%d-%H.%M.%S')))
parser.add_argument('-d,--device', dest='device', default=0, help='The video device number. Default: %(default)s')
parser.add_argument('--width',dest='width',default=640)
parser.add_argument('--height',dest='height',default=480)
parser.add_argument('--preview', action='store_true')

#ecto options
scheduler_options(parser.add_argument_group('Scheduler Options'))
args = parser.parse_args()

if not os.path.exists(args.output):
    os.makedirs(args.output)

#camera = VideoCapture(video_device=1,width=int(args.width),height=int(args.height))
source = create_source(package_name='image_pipeline', source_type='OpenNISource') #optionally pass keyword args here...
depth_saver = ImageSaver(filename_format=os.path.join(args.output, 'frame_%010d_depth.png'))
image_saver = ImageSaver(filename_format=os.path.join(args.output, 'frame_%010d_image.png'))
#mono_saver = ImageSaver(filename_format=os.path.join(args.output, 'frame_%010d_mono.png'))

plasm = ecto.Plasm()
plasm.connect(
#      camera['image'] >> imshow(name='Mono')['image'],
      source['depth'] >> imshow(name='Depth')['image'],
      source['image'] >> imshow(name='RGB')['image']
      )

if not args.preview:
    plasm.connect(
        source['image'] >> image_saver['image'],
    	source['depth'] >> depth_saver['image'],
#        camera['image'] >> mono_saver['image'],
    )

run_plasm(args, plasm, locals=vars())
