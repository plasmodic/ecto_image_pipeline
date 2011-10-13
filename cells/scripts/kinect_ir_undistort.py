#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv.highgui import V4LCapture, VideoCapture, imshow, FPSDrawer, NiConverter
from image_pipeline import Rectifier, RectifierNC, StereoModelLoader, DepthRegister
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma

capture = OpenNICapture(stream_mode=IR, registration=False, latched=False)
stereo_model = StereoModelLoader(left_fname='left.yml', right_fname="right.yml", stereo_fname = 'stereo.yml');
rect_depth = Rectifier()
conversion = IRGamma() # scale the IR so that we can visualize it.


plasm = ecto.Plasm()
plasm.connect(
    capture['ir'] >> conversion[:],
    conversion[:] >> rect_depth['image'],
    stereo_model['right_model'] >> rect_depth['camera'],
    )


#display stuff
plasm.connect(
       rect_depth['image'] >> imshow(name='Rectified IR')['image'],
       conversion[:] >> imshow(name='Original IR')['image']
       )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect IR images and rectify.')
