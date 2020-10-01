"""This module describes the ObjectRecognitionServer."""

import numpy as np
import rospy
from cosypose.datasets.datasets_cfg import make_urdf_dataset
from geometry_msgs.msg import Point
from object_recognition_msgs.msg import ObjectInformation
from object_recognition_msgs.srv import (GetObjectInformation,
                                         GetObjectInformationResponse)
from shape_msgs.msg import Mesh, MeshTriangle

try:
    # tested on pyassimp=4.1.3
    from pyassimp import pyassimp
    use_pyassimp = True
except:
    try:
        import pyassimp
        use_pyassimp = True
    except:
        use_pyassimp = False

from .object_database import ObjectDatabase

__all__ = ('ObjectInformationService')


class ObjectInformationService:
    """The ObjectInformationServer class."""

    def __init__(self):
        """Constructor for a ObjectInformationServer.
        """
        self._datasets = {}
        self._meshes = {}

        self._service = rospy.Service(
            '/get_object_information', GetObjectInformation, self.execute)       

    def execute(self, req):
        """Action server callback.

        Callback gets called in a separate thread whenever
        a new goal is received.

        Args:
            req (GetObjectInformationRequest): service request

        Returns:
            GetObjectInformationResponse: object information            
        """
        try:
            db, key = req.type.db, req.type.key
            rospy.logdebug('Get information for %s/%s', db, key)

            if db not in self._datasets:
                self._datasets[db] = ObjectDatabase(db)
            dataset = self._datasets[db]

            if (db, key) not in self._meshes:
                self._meshes[(db, key)] = _make_mesh(
                    dataset.get_mesh_path(key),
                    dataset.get_mesh_scale(key))
            mesh = self._meshes[(db, key)]

            response = GetObjectInformationResponse()
            response.information.name = dataset.get_name(key)
            response.information.ground_truth_mesh = mesh
            return response
        except Exception as ex:
            rospy.logerr('Get information failed: %s', ex)
            rospy.logdebug(traceback.format_exc())
            raise ex


def _make_mesh(path, scale):
    """[summary]

    Arguments:
        path {str} -- path to the mesh file on disk
        scale {list} -- mesh scale x,y,z

    Raises:
        RuntimeError: pyassimp errors

    Returns:
        Mesh -- mesh message
    """
    rospy.logdebug('Make mesh: %s, scale: %.3f,%.3f,%.3f',  path, *scale)

    mesh = Mesh()

    if not use_pyassimp:
        raise RuntimeError('pyassimp is broken, cannot load meshes')

    scene = pyassimp.load(path)
    try:
        if not scene.meshes:
            raise RuntimeError('Unable to load mesh "{}"'.format(path))

        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if isinstance(face, np.ndarray):
                indices = face.tolist()
            else:
                indices = face.indices
            if len(indices) == 3:
                triangle.vertex_indices = list(indices)
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
    finally:
        pyassimp.release(scene)

    return mesh
