"""
Define a few generic functions for dealing with sources
"""
from ecto_openni import SXGA_RES, VGA_RES, FPS_15, FPS_30

CameraType = type('CameraType', (object,),
                   dict(RGBD='RGBD',
                        Monocular='Monocular',
                        Stereo='Stereo',
                        )
                   )

def load_source(m_name, source_type):
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


def create_source(package_name, source_type, **kwargs):
    """
    Loads a camera type from a given package to create a cell that is a camera source
    """
    import sys
    try:
        source_cls = load_source(package_name, source_type)
    except RuntimeError as e:
        print >> sys.stderr, e
        sys.exit(-1)

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
    res = {'vga': VGA_RES, 'sxga': SXGA_RES}
    fps = {'15': FPS_15, '30': FPS_30}

    def fps_converter(string):
        if string not in fps:
            raise RuntimeError('Invalid choice "%s": choose from %s' % (string, str(fps.keys())))
        return fps[string]

    def res_converter(string):
        if string not in res:
            raise RuntimeError('Invalid choice "%s": choose from %s' % (string, str(res.keys())))
        return res[string]

    group = parser.add_argument_group(group_name)
    group.add_argument('--fps', metavar='FPS', dest='fps', type=fps_converter,
                       default=fps['30'], help='The temporal resolution of the captured data')
    group.add_argument('--res', metavar='RES', dest='res', type=res_converter,
                       default=res['sxga'], help='The image resolution of the captured data.')

######################################################################################################
#testing user interface interface
def _assert_source_interface(cell):
    ''' This ensures that the given cell exhibits the minimal interface to be
    considered a source for object recognition
    '''
    outputs = dir(cell.outputs)
    #all sources must produce the following
    for x in ('K', 'image', 'depth'):
        if x not in outputs:
            raise NotImplementedError('The cell %s does not correctly implement the ' % cell +
                                      'source interface. Must have an output named %s' % x)
    #type checks
    for x in ('K', 'image', 'depth'):
        type_name = cell.outputs.at(x).type_name
        #TODO add more explicit types.
        if type_name != 'cv::Mat':
            raise NotImplementedError('This cell does not correctly implement the source interface.\n'
                                      'Must have an output named %s, with type %s\n'
                                      'This cells output at %s has type %s' % (x, 'cv::Mat', x, type_name))
    return cell
###########################################################################################################
