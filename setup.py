from setuptools import find_packages, setup

package_name = 'manipulability'

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
    maintainer='isrlab',
    maintainer_email='isrlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = manipulability.test_publisher:main',
            'banana=manipulability.banana:main',
            'depth_pixel_reader=manipulability.depth_pixel_reader:main',
            'aruco_depth_viewer=manipulability.aruco_depth_viewer:main',
            'aruco_global_pose_node=manipulability.aruco_global_pose_node:main',
            'aruco_detect=manipulability.aruco_detect:main',
            'simple_2detect=manipulability.simple_2detect:main',
            'extract_pointcloud=manipulability.extract_pointcloud:main',
        ],
    },
)
