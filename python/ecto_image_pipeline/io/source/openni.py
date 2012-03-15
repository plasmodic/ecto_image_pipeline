import ecto

from ecto_opencv.calib import DepthTo3d, DepthMask
from ecto_openni import OpenNICapture
from ecto_image_pipeline.base import RescaledRegisteredDepth, CameraFromOpenNI
from ecto_image_pipeline.io.source import CameraType

class OpenNISource(ecto.BlackBox):
    """
    """
    CAMERA_TYPE = CameraType.RGBD


    _depth_map = RescaledRegisteredDepth
    _points3d = DepthTo3d
    _source = OpenNICapture
    _depth_mask = DepthMask
    _converter = CameraFromOpenNI

    def declare_params(self, p):
        p.forward_all('_source')

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_source', cell_key='image',
                  doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_depth_map', cell_key='depth',
                  doc='The depth map from a OpenNI device.' +
                  ' This is a CV_32FC1, with values in meters.')
        o.forward('depth_raw',cell_name='_source',cell_key='depth',
                        doc='The 16bit depth image.')
        o.forward('K', cell_name='_source', cell_key='K',
                  doc='The camera intrinsics matrix.')
        o.forward('points3d', cell_name='_points3d')
        o.forward('mask', cell_name='_depth_mask')
        o.forward('camera', cell_name='_converter')

    def connections(self):
        #rescaling ...
        graph = [self._source['depth'] >> self._depth_map['depth'],
                  self._source['image'] >> self._depth_map['image'],
                  ]

        #depth ~> 3d calculations
        graph += [
                  self._depth_map['depth'] >> self._points3d['depth'],
                  self._source['K'] >> self._points3d['K'],
                  self._depth_map['depth'] >> self._depth_mask['depth']
                 ]

        keys = ('depth', 'image', 'focal_length_image', 'focal_length_depth', 'baseline')
        #camera model
        graph += [ self._source[keys] >> self._converter[keys] ]

        return graph
