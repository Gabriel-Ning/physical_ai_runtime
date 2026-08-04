from setuptools import find_packages, setup

package_name = 'marker_detectors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel-Ning',
    maintainer_email='guomning@gmail.com',
    description='ROS 2 Python marker detection nodes (ChArUco, ArUco) for hand-eye calibration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'charuco_detector_node = marker_detectors.charuco_detector_node:main',
            'aruco_detector_node = marker_detectors.aruco_detector_node:main',
        ],
    },
)
