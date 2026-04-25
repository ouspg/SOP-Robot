from glob import glob

from setuptools import find_packages, setup

package_name = "tts_package"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Konsta Laurila",
    maintainer_email="klaurila20@student.oulu.fi",
    description="ROS2 text-to-speech playback node for SOP Robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tts_node = tts_package.tts_node:main",
            "tts_client = tts_package.tts_member_function:main",
        ],
    },
)
