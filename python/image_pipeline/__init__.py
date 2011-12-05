import platform, sys

if platform.system().startswith('freebsd'):
        # C++ modules are extremely fragile when loaded with RTLD_LOCAL,
        # which is what Python uses on FreeBSD by default, and maybe other
        # systems. Convince it to use RTLD_GLOBAL.

        # See thread by Abrahams et al:
        # http://mail.python.org/pipermail/python-dev/2002-May/024074.html
        sys.setdlopenflags(0x102)

def load_pybindings(name, path):
    """
    Merges python bindings from shared library 'name' into module 'name'.
    Use when you have a directory structure::

      lib/
         foo.so
         foo/
           __init__.py
           something.py

    Here, inside ``foo/__init__.py`` call ``load_pybindings(__name__, __path__)``

    this assumes that the first entry in list ``__path__`` is where
    you want the wrapped classes to merge to.

    """

    import imp
    m = imp.load_dynamic(name, path[0] + ".so") #TODO this is only going to work on unix...
    thismod = sys.modules[name]

    for (k,v) in m.__dict__.items():
        if not k.startswith("_"):
            thismod.__dict__[k] = v

load_pybindings(__name__, __path__)

from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)


