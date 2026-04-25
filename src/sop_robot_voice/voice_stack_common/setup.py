from glob import glob

from setuptools import find_packages, setup

package_name = "voice_stack_common"

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
        (f"share/{package_name}/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aapo Pihlajaniemi",
    maintainer_email="aapoto1201@gmail.com",
    description="Shared config, contracts, and runtime helpers for the SOP Robot voice stack.",
    license="MIT",
)
