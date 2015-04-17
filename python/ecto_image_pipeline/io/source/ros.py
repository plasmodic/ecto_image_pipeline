"""
Module defining several inputs for the object recognition pipeline
"""
from .camera_base import CameraType
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from ecto_image_pipeline.base import RescaledRegisteredDepth
try:
    from ecto_image_pipeline.conversion import MatToPointCloudXYZOrganized
    HAS_PCL = True
except:
    HAS_PCL = False
from ecto_opencv.calib import DepthTo3d, CropBox
import ecto
import ecto_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

########################################################################################################################

class BaseSource(ecto.BlackBox):
    """
    A source that uses ROS to produce image, depth, and camera info.
    The implementer needs to declare the "source" cell in declare_cells
    """
    #_depth_mask = DepthMask

    CAMERA_TYPE = CameraType.RGBD

    @staticmethod
    def fps_translate(fps):
        return None

    @staticmethod
    def res_translate(res):
        return None

    @staticmethod
    def declare_cells(_p):
        res = {'camera_info_image': CellInfo(ecto_ros.CameraInfo2Cv),
                'camera_info_depth': CellInfo(ecto_ros.CameraInfo2Cv),
                'crop_box': CellInfo(CropBox),
                'depth_map': CellInfo(RescaledRegisteredDepth)}
        if HAS_PCL:
            res['cloud'] = CellInfo(MatToPointCloudXYZOrganized)
        return res

    @staticmethod
    def declare_forwards(_p):
        p = {'crop_box': 'all'}
        p['image'] = [Forward('topic_name', 'rgb_image_topic', 'The ROS topic for the RGB image.','/camera/rgb/image_color')]
        p['image_info'] = [Forward('topic_name', 'rgb_camera_info', 'The ROS topic for the RGB camera info.','/camera/rgb/camera_info')]
        p['depth'] = [Forward('topic_name', 'depth_image_topic','The ROS topic for the depth image.','/camera/depth_registered/image_raw')]
        p['depth_info'] = [Forward('topic_name', 'depth_camera_info','The ROS topic for the depth camera info.','/camera/depth_registered/camera_info')]

        i = {}
        o = {'camera_info_image': [Forward('K', 'K_image', 'The camera intrinsics matrix of the image camera.')],
             'depth_map': [Forward('K', 'K_depth', 'The camera intrinsics matrix of the depth camera.')],
             'crop_box': [Forward('rgb', 'image', 'The RGB image from a OpenNI device.'),
                          Forward('depth', new_doc='The depth map from a OpenNI device. This is a CV_32FC1, with values in meters.'),
                          Forward('points3d'), Forward('mask')]
            }
        if HAS_PCL:
            o['cloud'] = [Forward('point_cloud')]
        return (p,i,o)

    def configure(self, _p, _i, _o):
        #ROS message converters
        self._depth_converter = ecto_ros.Image2Mat()
        self._rgb_image = ecto_ros.Image2Mat(swap_rgb=True)

        #these transform the depth into something usable
        self._points3d = DepthTo3d()
        #self._depth_mask = BaseSource._depth_mask()

    def connections(self, p):
        #ros message converters
        graph = [self.source["image"] >> self._rgb_image["image"],
                  self.source["depth"] >> self._depth_converter["image"],
                  self.source["depth_info"] >> self.camera_info_depth['camera_info'],
                  self.source["image_info"] >> self.camera_info_image['camera_info']
                  ]

        #rescaling ...
        graph += [self._depth_converter['image'] >> self.depth_map['depth'],
                  self._rgb_image['image'] >> self.depth_map['image'],
                  self.camera_info_depth['K'] >> self.depth_map['K'],
                  ]

        #depth ~> 3d calculations
        graph += [
                  self.depth_map['depth'] >> self._points3d['depth'],
                  self.depth_map['K'] >> self._points3d['K'],
                  #self._depth_map['depth'] >> self._depth_mask['depth'],
                  #self._rgb_image['image'] >> self._cloud['image'],
                  self._points3d['points3d'] >> self.crop_box['points3d'],
                  self._rgb_image['image'] >> self.crop_box['rgb'],
                  self.depth_map['depth'] >> self.crop_box['depth']
                 ]
        if HAS_PCL:
            graph += [ self.crop_box['points3d'] >> self.cloud['points'] ]

        return graph

class OpenNISubscriber(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    @staticmethod
    def declare_cells(p):
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        qsize = 1
        cells = BaseSource.declare_cells(p)
        subs = dict(image=ImageSub(topic_name='/bogus_topic_image', queue_size=qsize),
                    image_info=CameraInfoSub(topic_name='/bogus_topic_image', queue_size=qsize),
                    depth=ImageSub(topic_name='/bogus_topic_depth', queue_size=qsize),
                    depth_info=CameraInfoSub(topic_name='/bogus_topic_depth', queue_size=qsize)
                 )
        cells.update(subs)
        cells['source'] = ecto_ros.Synchronizer('Synchronizator', subs=subs)

        return cells

    @staticmethod
    def declare_forwards(p_in):
        p, i, o = BaseSource.declare_forwards(p_in)

        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o['source'] = o.get('source',[]) + [Forward('image', 'image_message'),
                      Forward('depth', 'depth_message'), Forward('image_info', 'image_info_message'),
                      Forward('depth_info', 'depth_info_message')]

        return (p,i,o)

########################################################################################################################

class BagReader(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    @staticmethod
    def declare_cells(p):
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        cells = BaseSource.declare_cells(p)
        baggers = dict(image=ImageBagger(topic_name='/bogus_topic'),
                       depth=ImageBagger(topic_name='/bogus_topic'),
                       image_info=CameraInfoBagger(topic_name='/bogus_topic'),
                       depth_info=CameraInfoBagger(topic_name='/bogus_topic'),
                       )
        cells.update(baggers)
        cells['source'] = ecto_ros.BagReader('Bag Reader', bag='/bogus', baggers=baggers)

        return cells

    @staticmethod
    def declare_forwards(p_in):
        p, i, o = BaseSource.declare_forwards(p_in)
        p['source'] = [Forward('bag', 'bag', "The bag file name.", "data.bag")]

        o['source'] = [Forward('image', 'image_message'), Forward('depth', 'depth_message'),
                       Forward('image_info', 'image_info_message'), Forward('depth_info', 'depth_info_message')]

        return (p,i,o)
