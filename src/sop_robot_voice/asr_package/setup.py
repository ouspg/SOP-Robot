from glob import glob

from setuptools import find_packages, setup

package_name = "asr_package"

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
    description="ROS2 ASR package for the SOP Robot voice chatbot stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "asr_node = asr_package.asr_node:main",
        ],
    },
)
