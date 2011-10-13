#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from image_pipeline import Rectifier, RectifierNC, StereoModelLoader, DepthRegister, CameraModelToCv
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma
from ecto_object_recognition.conversion import MatToPointCloudXYZRGB
from ecto_pcl import PointCloudT2PointCloud, CloudViewer, XYZRGB

openni_reg = False
capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=openni_reg, latched=False)

stereo_model = StereoModelLoader(left_fname='left.yml', right_fname="right.yml", stereo_fname='stereo.yml');


plasm = ecto.Plasm()

if not openni_reg:
    rect_rgb = Rectifier()
    rect_depth = Rectifier()
    reg_depth = DepthRegister()

    plasm.connect(
        capture['image'] >> rect_rgb['image'],
        stereo_model['left_model'] >> rect_rgb['camera'],
        capture['depth'] >> rect_depth['image'],
        stereo_model['right_model'] >> rect_depth['camera'],
        # hook up the registration
        stereo_model['stereo_model'] >> reg_depth['rgbd_model'],
        rect_depth['image'] >> reg_depth['image']
        )

camera_converter = CameraModelToCv()
depthTo3d = DepthTo3d()
to_xyzrgb = MatToPointCloudXYZRGB()
pcl_cloud = PointCloudT2PointCloud(format=XYZRGB)
cloud_viewer = CloudViewer()

if not openni_reg:
    plasm.connect(
                  stereo_model['left_model'] >> camera_converter['camera'],
                  camera_converter['Kp'] >> depthTo3d['K'],
                  reg_depth['image'] >> depthTo3d['depth'],
                  depthTo3d['points3d'] >> to_xyzrgb['points'],
                  rect_rgb['image'] >> to_xyzrgb['image'],
                  to_xyzrgb[:] >> pcl_cloud[:],
                  pcl_cloud[:] >> cloud_viewer[:]
                  )
else:
    plasm.connect(
                  stereo_model['left_model'] >> camera_converter['camera'],
                  camera_converter['K'] >> depthTo3d['K'],
                  capture['depth'] >> depthTo3d['depth'],
                  depthTo3d['points3d'] >> to_xyzrgb['points'],
                  capture['image'] >> to_xyzrgb['image'],
                  to_xyzrgb[:] >> pcl_cloud[:],
                  pcl_cloud[:] >> cloud_viewer[:]
                  )
if not openni_reg:
    #display stuff
    plasm.connect(
           rect_rgb['image'] >> imshow(name='Rectified RGB')['image'],
           reg_depth['image'] >> imshow(name='Registered Depth')['image'],
           rect_depth['image'] >> imshow(name='Rectified Depth')['image'],

           capture['image'] >> imshow(name='Original')['image'],
           capture['depth'] >> imshow(name='Original Depth')['image']
           )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect depth and RGB and register them.')
