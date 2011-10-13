#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv.highgui import V4LCapture, VideoCapture, imshow, FPSDrawer, NiConverter
from image_pipeline import Rectifier, RectifierNC, StereoModelLoader, DepthRegister
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma

capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=False, latched=False)
stereo_model = StereoModelLoader(left_fname='left.yml', right_fname="right.yml", stereo_fname = 'stereo.yml');
rect_rgb = Rectifier()
rect_depth = Rectifier()
reg_depth = DepthRegister()


plasm = ecto.Plasm()
plasm.connect(
    capture['image'] >> rect_rgb['image'],
    stereo_model['left_model'] >> rect_rgb['camera'],
    capture['depth'] >> rect_depth['image'],
    stereo_model['right_model'] >> rect_depth['camera'],

    # hook up the registration
    stereo_model['stereo_model'] >> reg_depth['rgbd_model'],
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
