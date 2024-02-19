from setuptools import find_packages, setup

package_name = 'itabot_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carbon',
    maintainer_email='carbon225@protonmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = itabot_aruco.detector:main',
	    'camera_capture = itabot_aruco.capture_calibration_images_ros2:main',
        ],
    },
)
