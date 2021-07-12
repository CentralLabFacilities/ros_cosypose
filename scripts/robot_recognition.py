#!/usr/bin/env python3

import rospy
from ros_cosypose import ObjectRecognitionServer


def main():
    rospy.init_node('object_recognition')
    srv = ObjectRecognitionServer()
    srv.start()
    rospy.spin()


if __name__ == '__main__':
    main()
