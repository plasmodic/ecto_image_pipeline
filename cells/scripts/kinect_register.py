#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv.highgui import V4LCapture, VideoCapture, imshow, FPSDrawer, NiConverter
from image_pipeline import Rectifier, RectifierNC, CalibrationLoader, DepthRegister
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma

capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=False, latched=False)
cal_rgb = CalibrationLoader(filename='left.yml')
cal_depth = CalibrationLoader(filename='right.yml')
rect_rgb = Rectifier()
rect_depth = Rectifier()
reg_depth = DepthRegister()


plasm = ecto.Plasm()
plasm.connect(
    capture['image'] >> rect_rgb['image'],
    cal_rgb['camera'] >> rect_rgb['camera'],
    capture['depth'] >> rect_depth['image'],
    cal_depth['camera'] >> rect_depth['camera'],

    # hook up the registration
    cal_rgb['camera'] >> reg_depth['rgb_camera'],
    cal_depth['camera'] >> reg_depth['depth_camera'],
    rect_depth['image'] >> reg_depth['image']
    )

#display stuff
plasm.connect(
      rect_rgb['image'] >> imshow(name='Rectified RGB')['image'],
      rect_depth['image'] >> imshow(name='Rectified Depth')['image'],
      capture['image'] >> imshow(name='Original')['image'],
      capture['depth'] >> imshow(name='Original Depth')['image']
      )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect depth and RGB and register them.')
