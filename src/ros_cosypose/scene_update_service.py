"""This module describes the SceneUpdateService."""

import rospy
from moveit_msgs.msg import (CollisionObject, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.srv import GetObjectInformation

__all__ = ('SceneUpdateService')


class SceneUpdateService:
    """The SceneUpdateService class."""

    def __init__(self):
        """Constructor for a SceneUpdateService."""
        scene = self._get_planning_scene()
        objects = [co for co in scene.world.collision_objects
                   if co.id.startswith('recognized_object')]
        objects.sort(key=lambda co: co.id)
        self._cached_types = {i: co.type for i, co in enumerate(objects)}
        rospy.logdebug('Initial objects: %s', self._cached_types)

        self._apply_planning_scene = rospy.ServiceProxy(
            '/apply_planning_scene', ApplyPlanningScene)
        self._apply_planning_scene.wait_for_service(timeout=5.0)

        self._get_object_information = rospy.ServiceProxy(
            '/get_object_information', GetObjectInformation)
        self._apply_planning_scene.wait_for_service(timeout=5.0)

        self._sub = rospy.Subscriber(
            '/recognized_object_array', RecognizedObjectArray,
            self.recognized_object_cb, queue_size=1)

    def recognized_object_cb(self, roa):
        """Recognized object array message callback.

        Args:
            roa (RecognizedObjectArray): object list
        """
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        for i, ro in enumerate(roa.objects):
            co = CollisionObject(header=ro.header, type=ro.type)
            co.id = 'recognized_object_{:03d}'.format(i)
            co.mesh_poses.append(ro.pose.pose.pose)
            if ro.type == self._cached_types.get(i, None):
                co.operation = CollisionObject.MOVE
                rospy.logdebug('Move %s', co.id)
            else:
                mesh = self._get_object_mesh(ro.type)
                if mesh is None:
                    continue
                co.meshes.append(mesh)
                co.operation = CollisionObject.ADD
                rospy.logdebug('Add %s', co.id)
            scene.world.collision_objects.append(co)
            self._cached_types[i] = ro.type

        for i in range(len(roa.objects), len(self._cached_types)):
            co = CollisionObject()
            co.header.stamp = rospy.Time.now()
            co.id = 'recognized_object_{:03d}'.format(i)
            co.operation = CollisionObject.REMOVE
            rospy.logdebug('Remove %s', co.id)
            scene.world.collision_objects.append(co)
            del self._cached_types[i]

        resp = self._apply_planning_scene(scene)
        if not resp.success:
            rospy.logerr("Could not apply planning scene diff.")

    def _get_planning_scene(self):
        """Get planning scene.

        Returns:
            PlanningScene: current scene state
        """
        get_planning_scene = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)
        get_planning_scene.wait_for_service(timeout=5.0)

        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        try:
            resp = get_planning_scene(req)
            return resp.scene
        except rospy.ServiceException as ex:
            rospy.logerr('Failed to get initial planning scene: %s', ex)
            return PlanningScene()

    def _get_object_mesh(self, object_type):
        """Get object mesh by type.

        Args:
            object_type (str): object type (db+key)

        Returns:
            Mesh: mesh message
        """
        try:
            req = self._get_object_information(object_type)
            return req.information.ground_truth_mesh
        except rospy.ServiceException as ex:
            rospy.logerr('Could not get object information: %s', ex)


def main():
    rospy.init_node('scene_updater', log_level=rospy.DEBUG)
    service = SceneUpdateService()
    rospy.spin()


if __name__ == '__main__':
    main()
