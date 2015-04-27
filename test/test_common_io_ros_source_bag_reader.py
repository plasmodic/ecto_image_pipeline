#!/usr/bin/env python
from ecto_image_pipeline.io.source.ros import BagReader
from ecto_image_pipeline.io.source.camera_base import _assert_source_interface
import unittest

class TestSourceBagReader(unittest.TestCase):

    def test_interface(self):
        br = BagReader()

        print br.__doc__
        assert 'image_message' in br.__doc__
        assert 'depth' in br.__doc__
        assert 'image' in br.__doc__
        assert 'K' in br.__doc__

        #verify some types.
        assert 'boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const>' == br.outputs.at('image_message').type_name

        #this should pass our interface check
        _assert_source_interface(br)

        #test the bag file name parameter
        br = BagReader(bag='testy.bag')
        assert br.params.bag == 'testy.bag'
        assert br.source.params.bag == 'testy.bag'
        _assert_source_interface(br)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('ecto_image_pipeline', 'test_source_bag_reader', TestSourceBagReader)
