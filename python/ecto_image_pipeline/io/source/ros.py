"""
Module defining several inputs for the object recognition pipeline
"""
import ecto
import ecto_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_opencv.calib import DepthTo3d, DepthMask
from ecto_image_pipeline.base import RescaledRegisteredDepth
from ecto_image_pipeline.io.source import CameraType

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo


########################################################################################################################

class BaseSource(ecto.BlackBox):
    """A source that uses ROS to produce image, depth, and camera info.
    """
    _camera_info = ecto_ros.CameraInfo2Cv
    _rgb_image = ecto_ros.Image2Mat
    _depth_converter = ecto_ros.Image2Mat
    _depth_map = RescaledRegisteredDepth
    _depth_mask = DepthMask
    _points3d = DepthTo3d
    _source = None #this should be allocated in by implementers

    CAMERA_TYPE = CameraType.RGBD

    def declare_params(self, p):
        p.declare('rgb_image_topic','The ROS topic for the RGB image.','/camera/rgb/image_color')
        p.declare('rgb_camera_info','The ROS topic for the RGB camera info.','/camera/rgb/camera_info')
        p.declare('depth_image_topic','The ROS topic for the depth image.','/camera/depth_registered/image')
        p.declare('depth_camera_info','The ROS topic for the depth camera info.','/camera/depth_registered/camera_info')
        pass

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_rgb_image', cell_key='image', doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_depth_map', cell_key='depth', doc='The depth map from a OpenNI device. This is a CV_32FC1, with values in meters.')
        o.forward('K', cell_name='_camera_info', cell_key='K', doc='The camera intrinsics matrix.')
        o.forward('points3d', cell_name='_points3d')
        o.forward('mask', cell_name='_depth_mask')

    def configure(self, p, _i, _o):
        #ROS message converters
        self._depth_converter = BaseSource._depth_converter()
        self._camera_info = BaseSource._camera_info()
        self._rgb_image = BaseSource._rgb_image(swap_rgb=True)

        #these transform the depth into something usable
        self._depth_map = BaseSource._depth_map()
        self._points3d = BaseSource._points3d()
        self._depth_mask = BaseSource._depth_mask()

    def connections(self):
        #ros message converers
        graph = [self._source["image"] >> self._rgb_image["image"],
                  self._source["depth"] >> self._depth_converter["image"],
                  self._source["image_info"] >> self._camera_info['camera_info']
                  ]

        #rescaling ...
        graph += [self._depth_converter['image'] >> self._depth_map['depth'],
                  self._rgb_image['image'] >> self._depth_map['image'],
                  ]

        #depth ~> 3d calculations
        graph += [
                  self._depth_map['depth'] >> self._points3d['depth'],
                  self._camera_info['K'] >> self._points3d['K'],
                  self._depth_map['depth'] >> self._depth_mask['depth']
                 ]

        return graph

class OpenNISubscriber(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    def declare_io(self, p, i, o):
        BaseSource.declare_io(self, p, i, o)
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        #TODO Should these just be simple names where remapping is expected?
        qsize = 1
        subs = dict(image=ImageSub(topic_name=p.rgb_image_topic, queue_size=qsize),
                    image_info=CameraInfoSub(topic_name=p.rgb_camera_info, queue_size=qsize),
                    depth=ImageSub(topic_name=p.depth_image_topic, queue_size=qsize),
                    depth_info=CameraInfoSub(topic_name=p.depth_camera_info, queue_size=qsize)
                 )
        #Creating this in declare io, so that i can declare the output with a concrete type.
        self._source = ecto_ros.Synchronizer('Synchronizator', subs=subs)
        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o.declare('image_message', self._source.outputs.at('image'))
        o.declare('depth_message', self._source.outputs.at('depth'))
        o.declare('image_info_message', self._source.outputs.at('image_info'))
        o.declare('depth_info_message', self._source.outputs.at('depth_info'))

########################################################################################################################

class BagReader(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    def declare_params(self, p):
        BaseSource.declare_params(self, p)
        p.declare('bag', "The bag file name.", "data.bag")

    def declare_io(self, p, i, o):
        BaseSource.declare_io(self, p, i, o)
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        #TODO Should these just be simple names where remapping is expected?
        baggers = dict(image=ImageBagger(topic_name=p.rgb_image_topic),
                       depth=ImageBagger(topic_name=p.depth_image_topic),
                       image_info=CameraInfoBagger(topic_name=p.rgb_camera_info),
                       depth_info=CameraInfoBagger(topic_name=p.depth_camera_info),
                       )
        #Creating this in declare io, so that i can declare the output with a concrete type.
        self._source = ecto_ros.BagReader('Bag Reader', bag=p.bag, baggers=baggers)
        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o.declare('image_message', self._source.outputs.at('image'))
        o.declare('depth_message', self._source.outputs.at('depth'))
        o.declare('image_info_message', self._source.outputs.at('image_info'))
        o.declare('depth_info_message', self._source.outputs.at('depth_info'))
