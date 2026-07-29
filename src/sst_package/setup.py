from glob import glob

from setuptools import setup

package_name = 'sst_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aapo Pihlajaniemi',
    maintainer_email='apihlaja20@student.oulu.fi',
    description='Finnish speech recognition with local Faster Whisper.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'sst_node = sst_package.sst_node:main',
        ],
    },
)
