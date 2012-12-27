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
