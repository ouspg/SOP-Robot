from setuptools import setup, find_packages

package_name = "core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["core", "core.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Henrik Peteri & Petra Heikkinen",
    maintainer="na",
    maintainer_email="na@example.com",
    description="Core module for SOP-Robot",
    license="Apache License 2.0",
    entry_points={
        'console_scripts': [
        ],
    },
)
