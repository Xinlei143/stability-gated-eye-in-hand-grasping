import types
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

from foam_grasp import foam_camera_to_base_node as camera_to_base


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Buffer:
    def __init__(self, transform=None, error=None):
        self.transform = transform
        self.error = error
        self.calls = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.error is not None:
            raise self.error
        return self.transform


def _tf_node(buffer):
    node = object.__new__(camera_to_base.FoamCameraToBaseNode)
    node.transform_source = "tf"
    node.base_frame = "base_link"
    node.tf_timeout = 0.2
    node.tf_buffer = buffer
    node.latest_base_points = {}
    node.last_warning_time = 0.0
    node.point_publishers = {1: _CapturePublisher()}
    node.marker_publisher = _CapturePublisher()
    node.get_logger = lambda: types.SimpleNamespace(warning=Mock())
    return node


class CameraToBaseTransformTest(unittest.TestCase):
    def test_tf_mode_looks_up_message_frame_at_message_timestamp(self):
        message = PointStamped()
        message.header.frame_id = "camera_color_optical_frame"
        message.header.stamp = RosTime(sec=4, nanosec=5)
        message.point.x = 0.1
        message.point.y = -0.2
        message.point.z = 0.7
        transformed = PointStamped()
        transformed.point.x = 1.1
        transformed.point.y = 1.2
        transformed.point.z = 1.3
        buffer = _Buffer(transform=object())
        node = _tf_node(buffer)

        with patch.object(
            camera_to_base,
            "do_transform_point",
            return_value=transformed,
            create=True,
        ) as do_transform:
            node.point_callback(1, message)

        self.assertEqual(len(buffer.calls), 1)
        args, kwargs = buffer.calls[0]
        self.assertEqual(args[0], "base_link")
        self.assertEqual(args[1], "camera_color_optical_frame")
        self.assertEqual(args[2], Time.from_msg(message.header.stamp))
        self.assertIsInstance(kwargs["timeout"], Duration)
        self.assertAlmostEqual(kwargs["timeout"].nanoseconds / 1e9, 0.2)
        do_transform.assert_called_once_with(message, buffer.transform)
        output = node.point_publishers[1].messages[0]
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertEqual(
            (output.point.x, output.point.y, output.point.z),
            (1.1, 1.2, 1.3),
        )

    def test_tf_mode_does_not_publish_when_transform_is_missing(self):
        message = PointStamped()
        message.header.frame_id = "camera_color_optical_frame"
        buffer = _Buffer(error=TransformException("no transform"))
        node = _tf_node(buffer)

        node.point_callback(1, message)

        self.assertEqual(len(buffer.calls), 1)
        self.assertEqual(node.point_publishers[1].messages, [])
        self.assertEqual(node.marker_publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
