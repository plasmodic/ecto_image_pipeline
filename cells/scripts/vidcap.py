#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from image_pipeline import Rectifier, CalibrationLoader

video_cap = VideoCapture(video_device=0, width=640, height=480)

calibration = CalibrationLoader(file='calibration.yml')
rectifier = Rectifier()

plasm = ecto.Plasm()

#pipleline
plasm.connect(video_cap['image'] >> rectifier['image'],
              )

#display stuff
plasm.connect(
              rectifier['image'] >> imshow(name='Rectified')['image'],
              video_capt['image'] >> imshow(name='Original')['image']
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.')
