from setuptools import find_packages, setup

package_name = 'surface_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/detection.launch.py'
        ]),
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
            'talker = surface_detection.test_publisher:main',
            'banana=surface_detection.banana:main',
            'depth_pixel_reader=surface_detection.depth_pixel_reader:main',
            'aruco_depth_viewer=surface_detection.aruco_depth_viewer:main',
            'aruco_global_pose_node=surface_detection.aruco_global_pose_node:main',
            'aruco_detect=surface_detection.aruco_detect:main',
            'simple_2detect=surface_detection.simple_2detect:main',
            'extract_pointcloud=surface_detection.extract_pointcloud:main',
            'capsule_pointcloud=surface_detection.capsule_pointcloud:main',
        ],
    },
)
