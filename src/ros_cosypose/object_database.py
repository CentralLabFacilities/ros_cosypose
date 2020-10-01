"""This module describes the ObjectDatabase."""

from cosypose.datasets.datasets_cfg import make_urdf_dataset


__all__ = ('ObjectDatabase')


class ObjectDatabase:
    """The ObjectDatabase class."""

    def __init__(self, dataset_name):
        """Constructor for a ObjectDatabase.

        Args:
            dataset_name (str): dataset name
        """
        self._dataset_info = {}
        for urdf_info in make_urdf_dataset(dataset_name):
            label = urdf_info['label']
            mesh_path = urdf_info['urdf_path'].replace('.urdf', '.obj')
            mesh_scale = float(urdf_info['scale'])
            self._dataset_info[label] = (mesh_path, mesh_scale)

    def get_name(self, label):
        """Return human readable name for an object.

        Arguments:
            label {str} -- unique object id

        Returns:
            str -- name      
        """
        return label

    def get_mesh_path(self, label):
        """Return path to an object mesh on disk.

        Arguments:
            label {str} -- unique object id

        Returns:
            str -- absolute mesh path
        """
        return self._dataset_info[label][0]

    def get_mesh_scale(self, label):
        """Return an object mesh scale.

        Arguments:
            label {str} -- unique object id

        Returns:
            list -- mesh scale x,y,z
        """
        return [self._dataset_info[label][1]] * 3
