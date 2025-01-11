import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pi_ros2_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.xml')),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    maintainer='koh',
    maintainer_email='sktlgt93@gmail.com',
    description='ROS2 Package for Raspberry Pi Camera Modules using OpenCV',
    license='MIT License',
    entry_points={
        'console_scripts': [
            # picam
            'pub_picam_raw  = pi_ros2_cv.picam.pub_picam_raw:main',
            # misc
            'image_view     = pi_ros2_cv.misc.image_view:main',
        ],
    },
)
