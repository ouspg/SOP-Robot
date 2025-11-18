from setuptools import setup, find_packages
from glob import glob
import os

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["perception", "perception.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        (os.path.join("share", package_name, "face_detection", "models"),
            glob("perception/face_detection/models/*")),

        (os.path.join("share", package_name, "face_detection", "predictors"),
            glob("perception/face_detection/predictors/*")),

        (os.path.join("share", package_name, "face_detection", "launch"),
            glob("perception/face_detection/launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vagrant",
    maintainer_email="na@example.com",
    description="Package for SOP-Robot Perception",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_detection = perception.face_detection.node.face_detection_node:main",
        ],
    },
)
