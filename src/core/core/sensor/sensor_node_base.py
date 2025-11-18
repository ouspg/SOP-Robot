"""
sensor_node_base.py

This module containts the base class 'SensorNodeBase' for Sensor Nodes.
It defines the common interface and initialization for all sensor node implementations
"""

from typing import Type, Dict, Any
from rclpy.node import Node
from .sensor_base import SensorBase

class SensorNodeBase(Node):
    """
    Base class for all sensor-related nodes in the SOP-Robot.
    """

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.node_name: str = node_name
        self.sensors: Dict[str, SensorBase] = {}
        self.get_logger().info(f"Sensor Node [{node_name}] initialized.")

    def add_sensor(self, sensor_name: str, sensor_class: Type[SensorBase], \
                   *args: Any, **kwargs: Any) -> None:
        """
        Dynamically add a sensor to the node's attributes by setattr
        """

        if sensor_name not in self.sensors:
            sensor = sensor_class(sensor_name, self, *args, **kwargs)
            setattr(self, sensor_name, sensor)
            self.sensors[sensor_name] = sensor
            self.get_logger().warn(f"Sensor [{sensor_name}] added to Node [{self.node_name}].")
        else:
            self.get_logger().warn(f"Sensor [{sensor_name}] already exists in Node [{self.node_name}].")

    def read_sensors(self) -> None:
        """
        Read all sensors in the node
        """
        for sensor_name in self.sensors:
            sensor_instance = getattr(self, sensor_name)
            if sensor_instance:
                sensor_instance.read()
