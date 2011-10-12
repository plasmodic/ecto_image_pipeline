#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID, GatherPoints
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma
from image_pipeline import StereoCalibration

from pattern_helpers import *

create_detector_drawer = create_detector_drawer_chessboard

ir_reader = ImageReader(path='ir')
rgb_reader = ImageReader(path='rgb')

plasm = ecto.Plasm()

ir_detector, ir_drawer = create_detector_drawer()
rgb_detector, rgb_drawer = create_detector_drawer()

rgb_display = imshow(name="RGB Points")

plasm.connect(ir_reader['image'] >> (ir_detector[:], ir_drawer['input']),
              rgb_reader['image'] >> (rgb_detector[:], rgb_drawer['input']),
              ir_detector['found', 'out'] >> ir_drawer['found', 'points'],
              rgb_detector['found', 'out'] >> rgb_drawer['found', 'points'],
              rgb_drawer[:] >> rgb_display[:],
              ir_drawer[:] >> imshow(name='Depth points')[:]
              )

#triggers
ander = ecto.And(ninput=2)
plasm.connect(rgb_detector['found'] >> ander['in1'],
              ir_detector['found'] >> ander['in2'],
            )

calibrator = ecto.If('calibrator',cell=StereoCalibration())

plasm.connect(rgb_detector['out', 'ideal'] >> calibrator['points_left', 'points_object'],
              ir_detector['out'] >> calibrator['points_right'],
              ander[:] >> calibrator['__test__'],
              rgb_reader['image'] >> calibrator['image']
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()
print sched.stats()
