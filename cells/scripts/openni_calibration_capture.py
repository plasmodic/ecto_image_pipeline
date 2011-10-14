#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageSaver
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID, CHESSBOARD, GatherPoints
from ecto_opencv.imgproc import cvtColor, RGB2GRAY
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma
from image_pipeline import StereoCalibration, LatchBool
import os

from pattern_helpers import *

create_detector_drawer = create_detector_drawer_chessboard
capture = OpenNICapture(stream_mode=RGB, registration=False, latched=True)
next_mode = IR
conversion = IRGamma() # scale the IR so that we can visualize it.
rgb2gray = cvtColor(flag=RGB2GRAY)

plasm = ecto.Plasm()
plasm.insert(capture)
plasm.connect(capture['image'] >> rgb2gray[:],
              capture['ir'] >> conversion[:],
              )

ir_detector, ir_drawer = create_detector_drawer()
rgb_detector, rgb_drawer = create_detector_drawer()

rgb_display = imshow(name="RGB Points", triggers=dict(save=ord('s')))
trigger_latch = LatchBool()
reset_source, reset_sink = ecto.EntangledPair(value=trigger_latch.inputs.at('reset'), source_name='Reset Source', sink_name='Reset Sink')
plasm.connect(conversion[:] >> (ir_detector[:], ir_drawer['input']),
              rgb2gray[:] >> (rgb_detector[:], rgb_drawer['input']),
              ir_detector['found', 'out'] >> ir_drawer['found', 'points'],
              rgb_detector['found', 'out'] >> rgb_drawer['found', 'points'],
              rgb_drawer[:] >> rgb_display[:],
              ir_drawer[:] >> imshow(name='Depth points')[:],
              rgb_display['save'] >> (trigger_latch['input'], trigger_latch['set']),
              reset_source[:] >> trigger_latch['reset']
              )

#triggers
ander = ecto.And(ninput=3)
plasm.connect(rgb_detector['found'] >> ander['in1'],
              ir_detector['found'] >> ander['in2'],
              trigger_latch[:] >> ander['in3'],
              ander[:] >> reset_sink[:]
            )

def init_image_dir(prefix):
    if not os.path.exists(prefix):
        os.mkdir(prefix)
    else:
        files = sorted(os.listdir(prefix))
        files.reverse()#reverse lexographic order 
        for x in files:
            if '.png' in x:
                num = x.split('_')[1].split('.')[0]
                if num.isdigit():
                    return int(num) + 1
    return 0

rgb_start = init_image_dir('rgb')
ir_start = init_image_dir('ir')
print 'rgb start', rgb_start, 'ir_start', ir_start
if(rgb_start != ir_start):
    print 'please ensure that the images in ir, and rgb are aligned lexographically'
    sys.exit(1)
#saving images
rgb_saver = ecto.If(cell=ImageSaver(filename_format='rgb/rgb_%04d.png', start=rgb_start))
ir_saver = ecto.If(cell=ImageSaver(filename_format='ir/ir_%04d.png', start=ir_start))

plasm.connect(rgb2gray[:] >> rgb_saver['image'],
              conversion[:] >> ir_saver['image'],
              ander[:] >> (rgb_saver['__test__'], ir_saver['__test__'])
              )

sched = ecto.schedulers.Singlethreaded(plasm)
while sched.execute(niter=5) == 0:
    #swap it modes
    next_mode, capture.params.stream_mode = capture.params.stream_mode, next_mode

print sched.stats()
