from setuptools import setup

package_name = 'speech_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juha-Matti Runtti',
    maintainer_email='jruntti20@student.oulu.fi',
    description='Publisher/Subscriber speech recognition module with rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'talker = speech_python.publisher_member_function:main',
            #'listener = speech_python.subscriber_member_function:main'
            #'recognizer = speech_python.speech_rec:main'
            'speech_rec_server = speech_python.speech_rec_service:main',
            'speech_rec_client = speech_python.speech_rec_client_async:main'
        ],
    },
)
