#!/usr/bin/env python3

import rospy
from ros_cosypose.scene_update_service import SceneUpdateService


def main():
    rospy.init_node('scene_update')
    service = SceneUpdateService()
    rospy.loginfo('Scene update service is ready.')
    rospy.spin()


if __name__ == '__main__':
    main()
