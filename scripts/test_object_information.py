#!/usr/bin/env python

import rospy
from object_recognition_msgs.srv import (GetObjectInformation,
                                         GetObjectInformationRequest)


def main():
    rospy.init_node('test_object_information')
    rospy.wait_for_service('/get_object_information')
    try:
        get_object_information = rospy.ServiceProxy(
            '/get_object_information', GetObjectInformation)

        request = GetObjectInformationRequest()
        request.type.db = 'ycbv'
        request.type.key = 'obj_000003'

        response = get_object_information(request)
        rospy.loginfo('Object has name %s, mesh contains %d vertices and %d triangles',
                      response.information.name,
                      len(response.information.ground_truth_mesh.vertices),
                      len(response.information.ground_truth_mesh.triangles))
    except rospy.ServiceException as ex:
        rospy.logerr('Service call failed: %s', ex)


if __name__ == '__main__':
    main()
