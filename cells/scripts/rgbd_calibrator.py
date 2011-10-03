#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, VideoCapture
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID, CHESSBOARD
from ecto_opencv.imgproc import cvtColor, Conversion
from image_pipeline import CalibImageWriter
import sys
rows = 5
cols = 3
square_size = 0.04 #4 cm
#pattern_type = ASYMMETRIC_CIRCLES_GRID
pattern_type = CHESSBOARD
n_obs = 50
calibration_file = 'camera_new.yml'
image_file = 'rgb-00.png'

video = VideoCapture(video_device=0)
rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)
pattern_detector = PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type, square_size=square_size)
camera_calibrator = CameraCalibrator(output_file_name=calibration_file, n_obs=n_obs)
image_writer = CalibImageWriter(filename=image_file)
pattern_drawer = PatternDrawer(rows=rows, cols=cols)

plasm = ecto.Plasm()
plasm.connect(video['image'] >> (pattern_drawer['input'], image_writer['image'], rgb2gray['image']),
              rgb2gray['image'] >> pattern_detector['input'],
              pattern_drawer['out'] >> imshow(name='pattern')['image'],
              pattern_detector['found'] >> image_writer['found'],
              pattern_detector['out', 'found'] >> pattern_drawer['points', 'found'],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()

