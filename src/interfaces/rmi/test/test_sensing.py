from types import SimpleNamespace

import pytest
from rclpy.qos import qos_profile_sensor_data
from rmi import Camera, CameraSensorConfig, Sensor
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64MultiArray


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = []
        self.now_s = 10.0

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def get_clock(self):
        sec = int(self.now_s)
        nanosec = round((self.now_s - sec) * 1e9)
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                nanoseconds=round(self.now_s * 1e9),
                to_msg=lambda: SimpleNamespace(sec=sec, nanosec=nanosec),
            )
        )


def _image(stamp_s: int, value: int) -> Image:
    message = Image()
    message.header.stamp.sec = stamp_s
    message.header.frame_id = "head_optical_frame"
    message.height = 1
    message.width = 1
    message.encoding = "mono8"
    message.step = 1
    message.data = [value]
    return message


def test_camera_preserves_source_receive_time_and_raw_message_without_copy():
    node = FakeNode()
    camera = Camera(
        CameraSensorConfig(name="head", ros_topic="/head/image"),
        node,
    )
    message = _image(7, 42)

    node.subscriptions[0].callback(message)
    frame = camera.frame

    assert frame.value is message
    assert frame.source_time_s == 7.0
    assert frame.receive_time_s == 10.0
    assert frame.frame_id == "head_optical_frame"
    assert frame.sequence == 1
    assert node.subscriptions[0].qos is qos_profile_sensor_data


def test_camera_jpeg_encoding_subscribes_to_compressed_image():
    node = FakeNode()
    camera = Camera(
        CameraSensorConfig(
            name="fisheye",
            ros_topic="/pika_fisheye/image/compressed",
            encoding="jpeg",
        ),
        node,
    )
    message = CompressedImage()
    message.header.stamp.sec = 7
    message.header.frame_id = "pika_fisheye_link"
    message.format = "jpeg"
    message.data = [0xFF, 0xD8, 0xFF, 0xD9]

    node.subscriptions[0].callback(message)
    frame = camera.frame

    assert node.subscriptions[0].message_type is CompressedImage
    assert frame.value is message
    assert frame.source_time_s == 7.0
    assert frame.frame_id == "pika_fisheye_link"


def test_camera_converter_and_bounded_history_are_applied_once_per_message():
    node = FakeNode()
    conversions = []
    camera = Camera(
        CameraSensorConfig(name="head", ros_topic="/head/image"),
        node,
        converter=lambda message: conversions.append(message.data[0]) or message.data[0],
        history_size=2,
    )

    for sequence in range(1, 4):
        node.now_s = 10.0 + sequence
        node.subscriptions[0].callback(_image(sequence, sequence))

    assert camera.value == 3
    assert conversions == [1, 2, 3]
    assert [sample.value for sample in camera.history] == [2, 3]


def test_generic_sensor_uses_receive_time_when_message_has_no_header():
    node = FakeNode()
    sensor = Sensor(
        name="wrench",
        node=node,
        topic="/wrench",
        message_type=Float64MultiArray,
        converter=lambda message: tuple(message.data),
    )
    message = Float64MultiArray(data=[1.0, 2.0])

    node.subscriptions[0].callback(message)

    assert sensor.value == (1.0, 2.0)
    assert sensor.latest.source_time_s == 10.0
    assert sensor.latest.receive_time_s == 10.0


def test_wait_next_times_out_and_close_destroys_subscription():
    node = FakeNode()
    sensor = Sensor(
        name="value",
        node=node,
        topic="/value",
        message_type=Float64MultiArray,
    )

    with pytest.raises(TimeoutError, match="next sensor sample"):
        sensor.wait_next(timeout=0.001)

    sensor.close()
    assert node.destroyed == [node.subscriptions[0]]


def test_wait_next_does_not_return_an_already_consumed_latest_sample():
    node = FakeNode()
    sensor = Sensor(
        name="value",
        node=node,
        topic="/value",
        message_type=Float64MultiArray,
    )
    node.subscriptions[0].callback(Float64MultiArray(data=[1.0]))

    with pytest.raises(TimeoutError, match="next sensor sample"):
        sensor.wait_next(timeout=0.001)
