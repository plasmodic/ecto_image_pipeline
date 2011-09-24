#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv.highgui import V4LCapture, VideoCapture, imshow, FPSDrawer, NiConverter
from image_pipeline import Rectifier, CalibrationLoader

cal_rgb = CalibrationLoader(filename='cal_rgb.yml')
cal_depth = CalibrationLoader(filename='cal_depth.yml')
rect_rgb = Rectifier()
rect_depth = Rectifier()

verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')

def kinect_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=False,
                   synchronize=False,
                   device=Device.KINECT
                   )

video_cap = VideoCapture(video_device=0, width=640, height=480)
#video_cap = V4LCapture(width=640, height=480)
kcap = kinect_vga(0);

plasm = ecto.Plasm()

#pipleline
plasm.connect(
    kcap[:] >> verter[:],
#    verter['image'] >> fps[:],
    verter['image'] >> rect_rgb['image'],
    cal_rgb['camera'] >> rect_rgb['camera'],      
    verter['depth'] >> rect_depth['image'],
    cal_depth['camera'] >> rect_depth['camera'],      
    )

#display stuff
plasm.connect(
    rect_rgb['image'] >> imshow(name='Rectified RGB')['image'],
    rect_depth['image'] >> imshow(name='Rectified Depth')['image'],
    verter['image'] >> imshow(name='Original')['image']
    )

#plasm.insert(kcap)

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.')
