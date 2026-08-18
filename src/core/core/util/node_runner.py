"""
node_runner.py

This module provides a helper function to start rclpy and run a Node
"""
import rclpy

def run_node(node_class):
    """
    Helper function to run a node
    """
    rclpy.init()
    node = node_class()
    
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
