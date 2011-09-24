#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv.highgui import V4LCapture, VideoCapture, imshow, FPSDrawer, NiConverter
from image_pipeline import Rectifier, RectifierNC, CalibrationLoader, DepthRegister

cal_rgb = CalibrationLoader(filename='cal_rgb.yml')
cal_depth = CalibrationLoader(filename='cal_depth.yml')
rect_rgb = Rectifier()
rect_depth = RectifierNC()
reg_depth = DepthRegister()

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

    # hook up the registration
    cal_rgb['camera'] >> reg_depth['rgb_camera'],
    cal_depth['camera'] >> reg_depth['depth_camera'],
    rect_depth['image'] >> reg_depth['image']
    )

#display stuff
# plasm.connect(
#      rect_rgb['image'] >> imshow(name='Rectified RGB')['image'],
#      rect_depth['image'] >> imshow(name='Rectified Depth')['image'],
#      verter['image'] >> imshow(name='Original')['image'],
#      reg_depth['image'] >> imshow(name='Registered Depth')['image']
#      )

#plasm.insert(kcap)

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect depth and RGB and register them.')
