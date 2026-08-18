"""
ROS2 package setup script to run inmoov_i2 package
"""

from setuptools import setup
from glob import glob
import os

package_name = "inmoov_i2"

setup(
    name=package_name,
    version="0.0.1",
    packages=["inmoov_i2",
              "inmoov_i2.i2eyes",
              "inmoov_i2.i2eyes.sensor"
              ],
        data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"),
            glob("inmoov_i2/launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Henrik Peteri & Petra Heikkinen",
    maintainer="na",
    maintainer_email="na@example.com",
    description="Inmoov i2 package",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "i2e_webcam = inmoov_i2.i2eyes.sensor.i2e_webcam_node:main"
        ],
    },
)
