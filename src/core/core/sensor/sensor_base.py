"""
sensor_base.py

This module contains the abstract base class 'SensorBase' for individual sensors.
It defines the common interface and initialization for all sensor implementation,
including the abstract method 'read', which must be implemented by subclasses.
"""

from abc import ABC, abstractmethod
from rclpy.node import Node


class SensorBase(ABC):
    """
    Abstract Base Class for individual sensors.
    """
    #pylint: disable=too-few-public-methods

    def __init__(self, sensor_name: str, node: Node) -> None:
        self.name: str = sensor_name
        self.node: Node = node

    @abstractmethod
    def read(self) -> None:
        """
        Read Sensor data and publish it.
        """
