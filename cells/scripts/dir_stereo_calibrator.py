#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID, GatherPoints
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma
from image_pipeline import StereoCalibration, Points2DAccumulator, Points3DAccumulator

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


left_points, right_points = ecto.If('Left Points', cell=Points2DAccumulator()), ecto.If('Right Points', cell=Points2DAccumulator())
object_points = ecto.If('Object Points', cell=Points3DAccumulator())

calibrator_ = StereoCalibration()
calibrator = ecto.If('calibrator', cell=calibrator_)

plasm.connect(rgb_detector['out'] >> left_points['points'],
              rgb_detector['ideal'] >> object_points['points'],
              ir_detector['out'] >> right_points['points'],
              ander[:] >> (left_points['__test__'], right_points['__test__'], object_points['__test__']),
              left_points['stacked'] >> calibrator['points_left'],
              object_points['stacked'] >> calibrator['points_object'],
              right_points['stacked'] >> calibrator['points_right'],
              rgb_reader['image'] >> calibrator['image']
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()

print "Calibrating"
calibrator_.process()
