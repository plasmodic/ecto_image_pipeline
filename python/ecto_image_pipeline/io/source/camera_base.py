"""
Define some base functions for dealing with cells interfacing with cameras
"""
import inspect
import types

CameraType = type('CameraType', (object,),
                   dict(RGBD='RGBD',
                        Monocular='Monocular',
                        Stereo='Stereo',
                        )
                   )

def create_source_class(m_name, source_type):
    import inspect
    import pkgutil
    '''
    Given a list of python packages, or modules, find all TrainingPipeline implementations.
    :param modules: A list of python package names, e.g. ['object_recognition']
    :returns: A list of TrainingPipeline implementation classes.
    '''
    if m_name == 'image_pipeline':
        if source_type == 'OpenNISource':
            from .openni import OpenNISource
            return OpenNISource
        if source_type == 'OpenNISubscriber':
            from .ros import OpenNISubscriber
            return OpenNISubscriber
        if source_type == 'BagReader':
            from .ros import BagReader
            return BagReader
    m = __import__(m_name)
    ms = [m]
    for loader, module_name, is_pkg in pkgutil.walk_packages(m.__path__):
        try:
            module = loader.find_module(module_name).load_module(module_name)
            ms.append(module)
        except ImportError as e:
            pass
    for pymodule in ms:
        for x in dir(pymodule):
            potential_source = getattr(pymodule, x)
            if inspect.isclass(potential_source) and potential_source.__name__ is source_type:
                return potential_source
    raise RuntimeError("No source found with type %s in package %s" % (source_type, m_name))


def create_source(package_name, source_type, outputs_list=['K_depth', 'K_image', 'image', 'depth'], **kwargs):
    """
    Loads a camera type from a given package to create a cell that is a camera source
    
    :param package_name: the Python package where to find the source
    :param source_type: the name of the cell to load
    :param outputs_list: a list of output tendrils the source should have. It has to be a subset of:
                K: the calibration matrix
                camera: a CameraInfo object
                image: the RGB image as a cv::Mat
                depth: the depth image as a cv::Mat (int16, uint16 or float)
                mask_depth: a mask of where the depth can be valid (useful for Kinect)
                points3d: a cv:Mat of CV_32FC3 representing the 3d  points
    """
    import sys
    try:
        source_cls = create_source_class(package_name, source_type)
    except RuntimeError as e:
        print >> sys.stderr, e
        sys.exit(-1)

    # translate the fps and res
    for val in ['fps', 'res']:
        if val in kwargs:
            if val == 'fps':
                val_new = source_cls.fps_translate(kwargs[val])
            elif val == 'res':
                val_new = source_cls.res_translate(kwargs[val])
            if val_new:
                kwargs[val] = val_new
            else:
                kwargs.pop(val)

    if outputs_list:
        # check the outputs
        allowed_outputs = set(['camera', 'depth', 'image', 'K_depth', 'K_image', 'mask_depth', 'points3d'])
        remain = set(outputs_list).difference(allowed_outputs)
        if remain:
            raise RuntimeError('Outputs are not part of the supported ones: %s' % str(remain))

        kwargs['outputs_list'] = outputs_list
        cell = source_cls('Source', **kwargs)
    else:
        cell = source_cls('Source', **kwargs)
    #make sure to check if this implements the camera interface.
    _assert_source_interface(cell)
    return cell

def add_camera_group(parser, group_name='camera'):
    """
    Add an argument group for resolution and fps of a camera

    :param parser: the parser to add a group to
    :param group_name: the name of the camera group
    """
    group = parser.add_argument_group(group_name)
    group.add_argument('--fps', metavar='FPS', dest='fps',
                       default='30', help='The temporal resolution of the captured data')
    group.add_argument('--res', metavar='RES', dest='res',
                       default='vga', choices=['vga','sxga'], help='The image resolution of the captured data.')

######################################################################################################
#testing user interface interface
def _assert_source_interface(cell):
    ''' This ensures that the given cell exhibits the minimal interface to be
    considered a source for object recognition
    '''
    # make sure we have converters for fps
    try:
        if not isinstance(cell.fps_translate, types.FunctionType):
            raise
    except AttributeError:
        raise NotImplementedError('Your source needs a method fps_translate that takes a string of the fps and '
                                  'translates to a type understandable by the constructor. If None is returned by '
                                  'the function, it will be ignored')
    try:
        if not isinstance(cell.res_translate, types.FunctionType):
            raise
    except AttributeError:
        raise NotImplementedError('Your source needs a method res_translate that takes a string of the resolution and '
                                  'translates to a type understandable by the constructor. If None is returned by '
                                  'the function, it will be ignored')

    outputs = dir(cell.outputs)
    #all sources must produce the following
    for x in ('K_depth', 'K_image', 'image', 'depth'):
        if x not in outputs:
            raise NotImplementedError('The cell %s does not correctly implement the ' % cell +
                                      'source interface. Must have an output named %s' % x)
    #type checks
    for x in ('K_depth', 'K_image', 'image', 'depth'):
        type_name = cell.outputs.at(x).type_name
        #TODO add more explicit types.
        if type_name != 'cv::Mat':
            raise NotImplementedError('This cell does not correctly implement the source interface.\n'
                                      'Must have an output named %s, with type %s\n'
                                      'This cells output at %s has type %s' % (x, 'cv::Mat', x, type_name))
    return cell
