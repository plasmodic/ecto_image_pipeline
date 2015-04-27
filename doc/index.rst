.. _ecto_image_pipeline:

ecto_image_pipeline: Camera Models, Calibration, etc.
=====================================================

Core C++ functionality for ecto_image_pipeline.

 * Camera Models
 * Calibration routines including writing and reading from disk
 * Camera interface conventions
 * Project routines
 * Rectification routines

Also has plugs for:
 * ROS
 * ecto

get it
^^^^^^
::

  % git clone git://github.com/wg-perception/ecto_image_pipeline.git

requires
^^^^^^^^
* boost
* cmake
* Eigen3 http://eigen.tuxfamily.org/
* OpenCV > 2.3 http://opencv.willowgarage.com/wiki/
* *optional* gtest
* *optional* ecto https://github.com/plasmodic/ecto

build it
^^^^^^^^
::

  % cd ecto_image_pipeline
  % mkdir build
  % cd build
  % cmake ..
  % make

test it
^^^^^^^
::

  % cd ecto_image_pipeline/build
  % make
  % ctest -V

use it
^^^^^^
See samples/ for projects that use ecto_image_pipeline



Cells
-----


.. ectomodule:: ecto_image_pipeline
