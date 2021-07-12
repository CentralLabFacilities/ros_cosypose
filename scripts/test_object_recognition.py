#!/usr/bin/env python3

import actionlib
import rospy
from object_recognition_msgs.msg import (ObjectRecognitionAction,
                                         ObjectRecognitionGoal)


def main():
    rospy.init_node('test_object_recognition')

    client = actionlib.SimpleActionClient(
        '/recognize_objects', ObjectRecognitionAction)
    client.wait_for_server()

    goal = ObjectRecognitionGoal()
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()

    rospy.loginfo('Recognized objects: %s',
                  ', '.join(['{}/{}: {:.2f}'.format(ro.type.db, ro.type.key, ro.confidence)
                             for ro in result.recognized_objects.objects]))


if __name__ == '__main__':
    main()
