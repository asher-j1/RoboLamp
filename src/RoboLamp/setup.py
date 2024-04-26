from setuptools import find_packages, setup

package_name = 'RoboLamp'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 5 CS391',
    maintainer_email='jasoni_a1@denison.edu',
    description='Controls the robotic lamp',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_gaze = RoboLamp.publisher_gaze:main',
            'subscriber_gaze = RoboLamp.subscriber_gaze:main',
            'move_servo = RoboLamp.move_servo:main',
        ],
    },
)
