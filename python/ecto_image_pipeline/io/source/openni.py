import ecto

from ecto_opencv.calib import DepthTo3d, DepthMask
from ecto_openni import OpenNICapture
from ecto_image_pipeline.base import RescaledRegisteredDepth, CameraFromOpenNI
from ecto_image_pipeline.io.source import CameraType

class OpenNISource(ecto.BlackBox):
    """
    """
    CAMERA_TYPE = CameraType.RGBD

    _source = OpenNICapture
    _depth_mask = DepthMask
    _converter = CameraFromOpenNI

    def declare_params(self, p):
        p.forward_all('_source')

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_source', cell_key='image',
                  doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_source', cell_key='depth',
                  doc='The depth map from a OpenNI device.' +
                  ' This is a CV_32FC1, with values in meters.')
        o.forward('depth_raw',cell_name='_source',cell_key='depth',
                        doc='The 16bit depth image.')
        o.forward('K', cell_name='_source', cell_key='K',
                  doc='The camera intrinsics matrix.')
        o.forward('mask_depth', cell_name='_depth_mask', cell_key='mask')
        o.forward('camera', cell_name='_converter')

    def connections(self):
        keys = ('depth', 'image', 'focal_length_image', 'focal_length_depth', 'baseline')
        #camera model
        graph = [ self._source[keys] >> self._converter[keys] ]

        return graph
