#!/usr/bin/env python

# Assume this script called from anaconda environment.
# Add conda site_packages path before ROS pathes.
import sys
from distutils.sysconfig import get_python_lib
sys.path.insert(1, get_python_lib())

import rospy
from ros_cosypose.object_recognition_server import ObjectRecognitionServer


def main():
    rospy.init_node('object_recognition')
    srv = ObjectRecognitionServer()
    srv.start()
    rospy.loginfo('Object recognition server is ready.')
    rospy.spin()

    # TODO: fix multiprocessing bugs in cosypose
    import psutil

    current_process = psutil.Process()
    for child in current_process.children(recursive=True):
        child.kill()


if __name__ == '__main__':
    main()
