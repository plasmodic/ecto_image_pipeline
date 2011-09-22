image_pipeline
==============
Core C++ functionality for image_pipeline.

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

  % git clone git://github.com/wg-perception/image_pipeline.git

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

  % cd image_pipeline
  % mkdir image_pipeline
  % cd build
  % cmake ..
  % make

test it
^^^^^^
::

  % cd image_pipeline/build
  % make test


