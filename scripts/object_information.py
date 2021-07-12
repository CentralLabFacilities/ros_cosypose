#!/usr/bin/env python3

# Assume this script called from anaconda environment.
# Add conda site_packages path before ROS pathes.
import sys
from distutils.sysconfig import get_python_lib
sys.path.insert(1, get_python_lib())

import rospy
from ros_cosypose.object_information_service import ObjectInformationService


def main():
    rospy.init_node('object_information')
    service = ObjectInformationService()
    rospy.loginfo('Object information service is ready.')
    rospy.spin()


if __name__ == '__main__':
    main()
