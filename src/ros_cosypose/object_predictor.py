"""This module describes the ObjectPredictor."""

import rospy

from cosypose.robot_pipeline.rigid_object_predictor import RigidObjectPredictor
from cosypose.robot_pipeline.utils import make_cameras


__all__ = ('ObjectPredictor')


class ObjectPredictor:
    """The ObjectPredictor class."""

    def __init__(self, camera_info):
        """Constructor for a ObjectPredictor.

        Args:
            camera_info (CameraInfo): intrinsic parameters of the camera
        """
        self._cameras = make_cameras([_get_intrinsics(camera_info)])
        self._scores = []

        one_instance_per_class = rospy.get_param(
            '~one_instance_per_class', True)
        detection_threshold = rospy.get_param(
            '~detection_threshold', 0.5)

        self._parameters = dict(
            one_instance_per_class=one_instance_per_class,
            detection_th=detection_threshold,
        )

        object_coarse_run_id = rospy.get_param(
            '~object_coarse_run_id', 'coarse-bop-ycbv-synt+real--822463')
        object_refiner_run_id = rospy.get_param(
            '~object_refiner_run_id', 'refiner-bop-ycbv-synt+real--631598')
        object_detector_run_id = rospy.get_param(
            '~object_detector_run_id', 'detector-bop-ycbv-synt+real--292971')

        self._predictor = RigidObjectPredictor(
            object_coarse_run_id=object_coarse_run_id,
            object_refiner_run_id=object_refiner_run_id,
            object_detector_run_id=object_detector_run_id,
        )

    @property
    def dataset_name(self):
        """Name of the dataset the predictor works on.
        """
        return self._predictor.pose_predictor.coarse_model.cfg.urdf_ds_name

    def predict(self, image, reset=True):
        """Predict objects type and position in the image.

        Args:
            image (numpy.ndarray): image in the RGB8 format
        """
        objects = self._predictor(
            [image], self._cameras, reset=reset, detector_kwargs=self._parameters)

        if reset:
            detections = self._predictor.detections
            self._scores = [info['score'] for _, info in detections.infos.iterrows()]
            self._bboxes = [bbox for bbox in detections.bboxes.cpu().numpy()]

        return [(obj['label'], obj['pose']) for obj in objects]

    def get_score(self, index):
        """Get score for i-th object.

        Arguments:
            index {int} -- object index

        Returns:
            float -- score 0..1
        """
        return self._scores[index]

    def get_bbox(self, index):
        """Get detected bounding box for i-th object.

        Arguments:
            index {int} -- object index

        Returns:
            tuple -- boundig box x1,y1,x2,y2
        """
        return self._bboxes[index]


def _get_intrinsics(camera_info):
    """Get intrinsic parameters from the camera info.

    Args:
        camera_info (CameraInfo): camera info message

    Returns:
        dict: intrinsics in the cosypose format
    """
    return dict(
        fx=camera_info.K[0], cx=camera_info.K[2],
        fy=camera_info.K[4], cy=camera_info.K[5],
        resolution=(camera_info.width, camera_info.height),
    )
