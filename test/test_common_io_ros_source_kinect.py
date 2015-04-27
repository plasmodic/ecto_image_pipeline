#!/usr/bin/env python
from ecto_image_pipeline.io.source.ros import OpenNISubscriber
from ecto_image_pipeline.io.source.camera_base import _assert_source_interface
import unittest

class TestSourceKinect(unittest.TestCase):

    def test_interfaces(self):
        kr = OpenNISubscriber()

        assert 'image_message' in kr.__doc__
        assert 'depth' in kr.__doc__
        assert 'image' in kr.__doc__
        assert 'K' in kr.__doc__

        #verify some types.
        assert 'boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const>' == kr.outputs.at('image_message').type_name

        #this should pass our interface check
        _assert_source_interface(kr)

        # Make sure we can reassign topics
        new_topic = '/overriden_depth_image_topic'
        kr = OpenNISubscriber(depth_image_topic=new_topic)
        assert kr.params.depth_image_topic == new_topic

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('ecto_image_pipeline', 'test_source_kinect', TestSourceKinect)
