"""
test_sensor.py

This module implements Unit Tests for SensorBase and SensorNodeBase modules
"""

import pytest
import rclpy
from rclpy.node import Node
from core.sensor import SensorNodeBase, SensorBase
from std_msgs.msg import String

class MockSensor(SensorBase):
    """
    Mock sensor to test SensorBase and SensorNodeBase functionality
    """
    #pylint: disable=too-few-public-methods

    def __init__(self, sensor_name: str, node: Node) -> None:
        super().__init__(sensor_name, node)
        self.is_read = False
        self.mock_data = "Mock sensor data"
        self.publisher = self.node.create_publisher(String, "mock_topic", 10)

    def read(self) -> None:
        self.node.get_logger().info(f"Mock published: {self.mock_data}")
        self.is_read = True


#pylint: disable=redefined-outer-name
@pytest.fixture
def ut_node():
    """
    Fixture to initialize a Node
    """
    rclpy.init()
    node = SensorNodeBase("UT_Node")
    yield node
    rclpy.shutdown()

def test_add_sensor(ut_node):
    """
    Test that sensor is added to the node
    """

    ut_node.add_sensor("mock_0", MockSensor)

    assert "mock_0" in ut_node.sensors
    assert isinstance(ut_node.mock_0, MockSensor)
    assert isinstance(ut_node.sensors["mock_0"], MockSensor)

def test_read_sensors(ut_node):
    """
    Test that sensors are read
    """

    ut_node.add_sensor("mock_0", MockSensor)
    ut_node.add_sensor("mock_1", MockSensor)

    ut_node.read_sensors()

    assert ut_node.mock_0.is_read
    assert ut_node.mock_1.is_read

def test_sensor_topic_name(ut_node):
    """
    Test that sensor has a publisher with correct topic name
    """

    ut_node.add_sensor("mock_0", MockSensor)
    assert ut_node.mock_0.publisher.topic_name == "/mock_topic"
