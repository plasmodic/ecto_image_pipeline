0.5.7 (2016-04-24)
------------------
* fix PCL compilation
* fix compilation with OpenCV3
* convert tests to proper nosetests
* move samples to the proper folder
* Contributors: Vincent Rabaud

0.5.6 (2015-04-19)
------------------
* fixing the depth rescale and its calibration matrix
* Contributors: nlyubova

0.5.5 (2015-04-06)
------------------
* Fixed issue with the format of images (now it is CV_32F)
* clean extensions
* do not depend on ecto_opencv for building
* convert a test to a proper gtest
* Contributors: Vincent Rabaud, nlyubova

0.5.4 (2014-11-20)
------------------
* add Eigen as a build dependency
* Contributors: Vincent Rabaud

0.5.3 (2014-11-20)
------------------
* remove the PCL dependency
* Contributors: Vincent Rabaud

0.5.2 (2014-04-13)
------------------
* re-add opencv_candidate as a dep as it is find_packaged
* Contributors: Vincent Rabaud

0.5.1 (2014-04-13)
------------------
* update dependencies
* Contributors: Vincent Rabaud

0.5.0 (2014-04-13)
------------------
* no need for g2o cells: g2o things should be done through the library anyway
* make the code compile on Indigo
* use the proper new camera topic
* use the newer catkin variable for tests
* Contributors: Vincent Rabaud

0.4.13 (2013-08-28 19:03:21 -0800)
----------------------------------
- fix dependencies with PCL
