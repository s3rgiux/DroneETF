# CMake generated Testfile for 
# Source directory: /home/domenico/dk_test/src/src/darknet_ros/darknet_ros
# Build directory: /home/domenico/dk_test/src/build/darknet_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_darknet_ros_rostest_test_object_detection.test "/home/domenico/dk_test/src/build/darknet_ros/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/domenico/dk_test/src/build/darknet_ros/test_results/darknet_ros/rostest-test_object_detection.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/domenico/dk_test/src/src/darknet_ros/darknet_ros --package=darknet_ros --results-filename test_object_detection.xml --results-base-dir \"/home/domenico/dk_test/src/build/darknet_ros/test_results\" /home/domenico/dk_test/src/src/darknet_ros/darknet_ros/test/object_detection.test ")
subdirs("gtest")
