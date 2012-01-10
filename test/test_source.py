#!/usr/bin/env python

import ecto
from ecto_opencv.highgui import imshow
from image_pipeline.io.source import create_source
from image_pipeline_conversion import MatToPointCloudXYZRGB
from ecto_pcl import PointCloudT2PointCloud, CloudViewer, XYZRGB
from ecto.opts import run_plasm, cell_options, scheduler_options
import argparse
import ecto_ros
import sys

parser = argparse.ArgumentParser(description='My awesome program thing.')

#ecto options
scheduler_options(parser)
args = parser.parse_args()

#add our cell to the parser
source = create_source(package_name='image_pipeline', source_type='OpenNISource') #optionally pass keyword args here...


to_xyzrgb = MatToPointCloudXYZRGB('cv::Mat ~> pcl::Cloud')
pcl_cloud = PointCloudT2PointCloud('pcl::Cloud ~> ecto::pcl::Cloud', format=XYZRGB)
cloud_viewer = CloudViewer('Cloud Display')

plasm = ecto.Plasm()
plasm.connect(source['points3d', 'image'] >> to_xyzrgb['points', 'image'],
              to_xyzrgb[:] >> pcl_cloud[:],
              pcl_cloud[:] >> cloud_viewer[:]
              )

run_plasm(args, plasm, locals=vars())
