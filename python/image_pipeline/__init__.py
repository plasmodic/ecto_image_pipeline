from pkgutil import extend_path
import platform
import sys
import os

if platform.system().startswith('freebsd'):
        # C++ modules are extremely fragile when loaded with RTLD_LOCAL,
        # which is what Python uses on FreeBSD by default, and maybe other
        # systems. Convince it to use RTLD_GLOBAL.

        # See thread by Abrahams et al:
        # http://mail.python.org/pipermail/python-dev/2002-May/024074.html
        sys.setdlopenflags(0x102)

def load_pybindings(name, input_path=None):
    """
    Has to be called from an __init__.py as follows:
    from object_recognition_core.utils.load_pybindings import load_pybindings
    __path__ = load_pybindings(__name__)

    If the input_path is provided, it will assume such structure (otherwise, it looks for foo.so anymore in the
    PYTHONPATH)

      lib/
         foo.so
         foo/
           __init__.py
           something.py

    Here, inside ``foo/__init__.py`` call ``load_pybindings(__name__, __path__)``

    this assumes that the first entry in list ``__path__`` is where
    you want the wrapped classes to merge to.
    """
    for path in os.environ['PYTHONPATH'].split(':'):
        if input_path:
            potential_additional_path = input_path
        else:
            potential_additional_path = os.path.join(path, *(name.split('.')))

        potential_additional_file = potential_additional_path + ".so"

        if not os.path.isfile(potential_additional_file):
            continue

        import imp
        m = imp.load_dynamic(name, potential_additional_file) #TODO this is only going to work on unix...
        thismod = sys.modules[name]

        for (k, v) in m.__dict__.items():
            if not k.startswith("_"):
                thismod.__dict__[k] = v

        if input_path:
            break

load_pybindings(__name__)

__path__ = extend_path(__path__, __name__)
