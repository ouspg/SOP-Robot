from setuptools import find_packages, setup

package_name = "full_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SOP Robot Team",
    maintainer_email="sop-robot@example.org",
    description="Launchable full-system demo package for SOP Robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "full_demo_node = full_demo.full_demo_node:main",
        ],
    },
)
