from glob import glob

from setuptools import find_packages, setup

package_name = "chatbot_app"

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
    maintainer="Aapo Pihlajaniemi",
    maintainer_email="aapoto1201@gmail.com",
    description="ROS2 GUI package for the SOP Robot voice chatbot stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "chatbot_app = chatbot_app.ros_app:main",
            "chatbot_app_unified = chatbot_app.unified_app:main",
            "chatbot_app_calibration = chatbot_app.calibration_app:main",
        ],
    },
)
