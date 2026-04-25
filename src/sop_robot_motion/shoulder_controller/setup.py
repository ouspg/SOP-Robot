from setuptools import find_packages, setup

package_name = "shoulder_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SOP Robot Team",
    maintainer_email="sop-robot@example.org",
    description="Unified fake and real shoulder controller bridge for SOP Robot.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "shoulder_controller_node = shoulder_controller.shoulder_controller_node:main",
        ],
    },
)
