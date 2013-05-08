"""
Module defining a source for an OpenNI device
"""

from ecto import BlackBox, BlackBoxCellInfo, BlackBoxForward
from ecto_image_pipeline.base import CameraFromOpenNI
from .camera_base import CameraType
from ecto_opencv.calib import DepthMask, DepthTo3d
from ecto_openni import OpenNICapture, SXGA_RES, VGA_RES, FPS_15, FPS_30

class OpenNISource(BlackBox):
    """
    """
    CAMERA_TYPE = CameraType.RGBD

    @staticmethod
    def fps_translate(fps):
        fps_choices = {'15': FPS_15, '30': FPS_30}
        if fps not in fps_choices:
            raise RuntimeError('fps not amongst the possible ones: %s' % str(fps_choices.keys()))
        return fps_choices[fps]

    @staticmethod
    def res_translate(res):
        res_choices = {'vga': VGA_RES, 'sxga': SXGA_RES}
        if res not in res_choices:
            raise RuntimeError('res not amongst the possible ones: %s' % str(res_choices.keys()))
        return res_choices[res]

    @classmethod
    def declare_cells(cls, p):
        cells = {'source': BlackBoxCellInfo(OpenNICapture)}
        if 'mask_depth' in p['outputs_list']:
            cells['depth_mask'] = BlackBoxCellInfo(DepthMask)
        if 'camera' in p['outputs_list']:
            cells['converter'] = BlackBoxCellInfo(CameraFromOpenNI)
        if 'points3d' in p['outputs_list']:
            cells['depth_to_3d'] = BlackBoxCellInfo(DepthTo3d)

        return cells

    @staticmethod
    def declare_direct_params(p):
        p.declare('outputs_list', 'A list of outputs to support', ['K', 'image', 'depth', 'mask_depth', 'camera', 'points3d'])

    @classmethod
    def declare_forwards(cls, p):
        cell_names = cls.declare_cells(p).keys()

        p = {'source': [BlackBoxForward('image_fps', 'fps'), BlackBoxForward('image_mode','res')]}
        o = {'source': [BlackBoxForward('image', '', 'The RGB image from a OpenNI device.'),
                        BlackBoxForward('depth', 'depth', 'The The 16bit depth image.'),
                        BlackBoxForward('K_depth', '', 'The camera intrinsics matrix.'),
                        BlackBoxForward('K_image', '', 'The camera intrinsics matrix.')]}
        if 'depth_mask' in cell_names:
            o['depth_mask'] = [BlackBoxForward('mask', 'mask_depth')]
        if 'converter' in cell_names:
            o['converter'] = [BlackBoxForward('camera')]
        if 'depth_to_3d' in cell_names:
            o['depth_to_3d'] = [BlackBoxForward('points3d')]

        return (p, {}, o)

    def connections(self, p):
        cell_names = self.declare_cells(p).keys()

        keys = ('depth', 'image', 'focal_length_image', 'focal_length_depth', 'baseline')
        graph = [ self.source ]

        if 'depth_mask' in cell_names:
            graph += [ self.source['depth'] >> self.depth_mask['depth'] ]
        if 'converter' in cell_names:
            graph += [ self.source[keys] >> self.converter[keys] ]
        if 'depth_to_3d' in cell_names:
            graph += [ self.source['depth', 'K_depth'] >> self.depth_to_3d['depth', 'K'] ]

        return graph
