"""This module describes the ObjectRecognitionServer."""

import traceback

import actionlib
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from object_recognition_msgs.msg import (ObjectRecognitionAction,
                                         ObjectRecognitionResult,
                                         RecognizedObject,
                                         RecognizedObjectArray)
from std_msgs.msg import ColorRGBA, Header
from transforms3d.quaternions import mat2quat
from visualization_msgs.msg import Marker, MarkerArray

from .image_listener import ImageListener
from .image_publisher import ImagePublisher
from .object_database import ObjectDatabase
from .object_predictor import ObjectPredictor

__all__ = ('ObjectRecognitionServer')


class ObjectRecognitionServer:
    """The ObjectRecognitionServer class."""

    def __init__(self, auto_start=False):
        """Constructor for a ObjectRecognitionServer.

        Args:
            auto_start (bool): start server
        """
        image_topic = rospy.get_param('~image')
        self._camera = ImageListener(image_topic)
        self._frame_id = self._camera.frame_id

        self._predictor = ObjectPredictor(self._camera.camera_info())
        self._dataset = ObjectDatabase(self._predictor.dataset_name)

        self._array_publisher = rospy.Publisher(
            '/recognized_object_array', RecognizedObjectArray, queue_size=1)

        self._marker_publisher = rospy.Publisher(
            '/recognized_object_marker_array', MarkerArray, queue_size=1)

        self._overlay_publisher = ImagePublisher(
            '/recognized_object_overlay', self._frame_id,
            self._camera.camera_info(), queue_size=1)

        self._server = actionlib.SimpleActionServer(
            '/recognize_objects', ObjectRecognitionAction,
            execute_cb=self.execute, auto_start=auto_start)

    def start(self):
        """Explicitly start the action server."""
        self._server.start()

    def execute(self, goal):
        """Action server callback.

        Callback gets called in a separate thread whenever
        a new goal is received.

        Args:
            goal (ObjectRecognitionGoal): action goal
        """
        if goal.use_roi:
            rospy.logwarn('ROI parameter is not supported.')

        try:
            image = self._camera.image(
                desired_encoding='rgb8', timeout=1.0)
            # TODO: enable tracking mode (reset=False)
            objects = self._predictor.predict(image, reset=True)

            rospy.logdebug('Recognized: %s', ','.join(
                [label for label, _ in objects]))

            # publish debug info
            self._publish_markers(objects)
            self._publish_overlay(image, objects)

            header = Header(stamp=rospy.Time.now(), frame_id=self._frame_id)
            roa = RecognizedObjectArray(header=header)

            for i, (label, pose) in enumerate(objects):
                ro = RecognizedObject(header=header)
                ro.type.db = self._predictor.dataset_name
                ro.type.key = label
                ro.confidence = self._predictor.get_score(i)
                ro.pose.header = header
                ro.pose.pose.pose = _pose_to_msg(pose)
                roa.objects.append(ro)

            self._array_publisher.publish(roa)

            result = ObjectRecognitionResult(recognized_objects=roa)
            self._server.set_succeeded(result)
        except Exception as ex:
            rospy.logerr('Recognition failed: %s', ex)
            rospy.logdebug(traceback.format_exc())
            self._server.set_aborted(text=str(ex))

    def _publish_markers(self, objects):
        """Publish debug markers.

        Args:
            objects (list): recognized objects         
        """
        if self._marker_publisher.get_num_connections() == 0:
            return

        marker_array = MarkerArray()

        marker = Marker(
            header=Header(stamp=rospy.Time.now(), frame_id=self._frame_id),
            action=Marker.DELETEALL,
        )
        marker_array.markers.append(marker)

        for i, (label, pose) in enumerate(objects):
            marker = Marker(
                header=Header(stamp=rospy.Time.now(), frame_id=self._frame_id),
                id=i,
                type=Marker.MESH_RESOURCE,
                action=Marker.MODIFY,
                pose=_pose_to_msg(pose),
                scale=Vector3(*self._dataset.get_mesh_scale(label)),
                color=ColorRGBA(1, 0, 1, 1),
                mesh_resource='file://' + self._dataset.get_mesh_path(label),
                mesh_use_embedded_materials=True,
            )
            marker_array.markers.append(marker)

        self._marker_publisher.publish(marker_array)

    def _publish_overlay(self, image, objects):
        """Publish debug detection boxes.

        Args:
            objects (list): recognized objects  
        """
        if self._overlay_publisher.get_num_connections() == 0:
            return

        import cv2
        font_scale = 1.0
        font = cv2.FONT_HERSHEY_PLAIN

        for i, (label, _) in enumerate(objects):
            text = '{}: {:.2f}'.format(label, self._predictor.get_score(i))
            x1, y1, x2, y2 = self._predictor.get_bbox(i).astype(int)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(image, text, (x1, y1), font,
                        font_scale, (255, 0, 0), 1)

        self._overlay_publisher.publish(image, encoding='rgb8')


def _pose_to_msg(matrix):
    return Pose(
            position = Point(*matrix[:3, 3]),
            orientation = Quaternion(*mat2quat(matrix[:3, :3]))
    )
