import ecto
from ecto import BlackBoxCellInfo, BlackBoxForward
from ecto_opencv.calib import DepthMask
from ecto_openni import OpenNICapture
from ecto_image_pipeline.base import CameraFromOpenNI
from ecto_image_pipeline.io.source import CameraType

class OpenNISource(ecto.BlackBox):
    """
    """
    CAMERA_TYPE = CameraType.RGBD

    @classmethod
    def declare_cells(cls, _p):
        return {'source': BlackBoxCellInfo(OpenNICapture),
                'depth_mask': BlackBoxCellInfo(DepthMask),
                'converter': BlackBoxCellInfo(CameraFromOpenNI)}

    @classmethod
    def declare_forwards(cls, _p):
        p = {'source': 'all'}
        o = {'source': [BlackBoxForward('image', '', 'The RGB image from a OpenNI device.'),
                                BlackBoxForward('depth', 'depth', 'The The 16bit depth image.'),
                                BlackBoxForward('depth', 'depth_raw', 'The The 16bit depth image.'),
                                BlackBoxForward('K', '', 'The camera intrinsics matrix.')],
                     'depth_mask': [BlackBoxForward('mask', 'mask_depth', '')],
                     'converter': [BlackBoxForward('camera', '', '')]}
        return (p,{},o)

    def connections(self, _p):
        keys = ('depth', 'image', 'focal_length_image', 'focal_length_depth', 'baseline')
        #camera model
        graph = [ self.source[keys] >> self.converter[keys],
                  self.source['depth'] >> self.depth_mask['depth'] ]

        return graph
