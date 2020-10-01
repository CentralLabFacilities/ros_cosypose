import rospy
import os
from sensor_msgs.msg import CameraInfo, Image

__all__ = ('ImagePublisher')


class ImagePublisher:
    """Wrapper to publish images to a topic."""

    def __init__(self, topic, frame_id, camera_info, queue_size=None):
        """[summary]

        Arguments:
            topic {str} -- image topic
            frame_id {str} -- camera frame_id
            camera_info (CameraInfo) -- camera info

        Keyword Arguments:
            queue_size {int} -- The queue size used for asynchronously (default: {None})
        """
        self._topic = topic
        self._frame_id = frame_id
        self._camera_info = camera_info
        self._image_pub = rospy.Publisher(
            topic + '/image', Image, queue_size=queue_size)
        self._camera_info_pub = rospy.Publisher(
            topic + '/camera_info', CameraInfo, queue_size=queue_size)

    def publish(self, image, encoding='passthrough'):
        """Publish image.

        Arguments:
            image {numpy.ndarray} -- image data

        Keyword Arguments:
            encoding {str} -- desired encoding (default: {'passthrough'})
        """
        msg = _array_to_imgmsg(image, encoding)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)
        self._camera_info_pub.publish(self._camera_info)

    def get_num_connections(self):
        """Get the number of connections to other ROS nodes for this topic.

        Returns:
            int -- number of connections
        """
        return self._image_pub.get_num_connections()


def _array_to_imgmsg(img_array, encoding):
    assert len(img_array.shape) == 3
    img_msg = Image()
    img_msg.height = img_array.shape[0]
    img_msg.width = img_array.shape[1]
    if encoding == 'passthrough':
        img_msg.encoding = '8UC3'
    else:
        img_msg.encoding = encoding
    if img_array.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    img_msg.data = img_array.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg
