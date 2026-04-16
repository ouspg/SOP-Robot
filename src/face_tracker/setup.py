from setuptools import setup
from glob import glob
import os

package_name = 'face_tracker'


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'predictors'), package_files('predictors/*')),
        (os.path.join('share', package_name, 'launch'), package_files('launch/*')),
        (os.path.join('share', package_name, 'models'), package_files('models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jari Jääskelä',
    maintainer_email='jari.jaaskela@oulu.fi',
    description='Face tracking, lip movement detection, and webcam publishing for SOP Robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_tracker_node = face_tracker.face_tracker_node:main',
            'mock_face_tracker_node = face_tracker.mock_face_tracker_node:main',
            'webcam_node = ' + package_name + '.webcam_node:main',
        ],
    },
)
