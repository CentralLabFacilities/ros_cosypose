"""This module describes the ObjectRecognitionServer."""

import traceback

import numpy as np
import rospy
import trimesh
from cosypose.datasets.datasets_cfg import make_urdf_dataset
from geometry_msgs.msg import Point
from object_recognition_msgs.msg import ObjectInformation
from object_recognition_msgs.srv import (GetObjectInformation,
                                         GetObjectInformationResponse)
from shape_msgs.msg import Mesh, MeshTriangle

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
                self._meshes[(db, key)] = self.make_mesh(
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

    @staticmethod
    def make_mesh(path, scale):
        """Load a mesh from disk and convert to a message.

        Arguments:
            path {str} -- path to the mesh file on disk
            scale {list} -- mesh scale x,y,z

        Returns:
            Mesh -- mesh message
        """
        rospy.logdebug('Make mesh: %s, scale: %.3f,%.3f,%.3f',  path, *scale)

        mesh = trimesh.load(path, force='mesh')
        return Mesh(
            vertices=[Point(*vertex) for vertex in mesh.vertices * scale],
            triangles=[MeshTriangle(vertex_indices=indices) for indices in mesh.faces])
