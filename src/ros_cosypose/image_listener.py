import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


__all__ = ('ImageListener')


class ImageListener:
    """Wrapper to asynchronously receiving latest Image from a topic.

    Useful when you capture data frequently.
    """

    def __init__(self, topic):
        """Constructor for an ImageListener.

        Arguments:
            topic {str} -- image topic to listen
        """
        self._topic = topic
        self._imgmsg = None

        def imgmsg_cb(msg):
            self._imgmsg = msg

        self._sub = rospy.Subscriber(topic, Image, imgmsg_cb, queue_size=1)

        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while not rospy.core.is_shutdown() and self._imgmsg is None:
            if rospy.Time.now() > deadline:
                rospy.logwarn_throttle(
                    1.0, 'Waiting for an image ({})...'.format(topic))
            rospy.rostime.wallsleep(0.01)

        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

    @property
    def frame_id(self):
        """Camera frame id

        Returns:
            str -- frame_id
        """
        image_message = self._imgmsg
        return image_message.header.frame_id

    def image(self, desired_encoding='passthrough', timeout=None):
        """Return latest received message as numpy array in specified encoding.

        Keyword Arguments:
            desired_encoding {str} -- one of 'rgb8', 'bgr8', 'F32C1' (default: {'passthrough'})
            timeout {float} -- image delay tolerance in seconds (default: {None})

        Raises:
            TimeoutError: image outdated

        Returns:
            numpy.ndarray -- image data
        """
        image_message = self._imgmsg
        if timeout is not None:
            if rospy.Time.now() > image_message.header.stamp + rospy.Duration(timeout):
                raise TimeoutError('Image "{}" outdated'.format(self._topic))
        return _imgmsg_to_array(image_message, encoding=desired_encoding)

    def camera_info(self, timeout=None):
        """Read the camera info for this stream.

        Keyword Arguments:
            timeout {float} -- time to wait in seconds (default: {None})

        Returns:
            CameraInfo -- camera info message
        """
        topic = '/'.join(self._topic.split('/')[:-1] + ['camera_info'])
        return rospy.wait_for_message(topic, CameraInfo, timeout=timeout)


def _imgmsg_to_array(msg, encoding):
    if msg.encoding in ['rgb8', 'bgr8'] and encoding in ['rgb8', 'bgr8', 'passthrough']:
        img_array = np.frombuffer(msg.data, dtype=np.uint8)
        img_array = img_array.reshape((msg.height, msg.width, 3))

        if encoding != 'passthrough' and encoding != msg.encoding:
            img_array = np.flip(img_array, axis=2)

        return img_array

    raise RuntimeError('Cannot convert {} image to {}'.format(
        msg.encoding, encoding))
