from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/line_follower.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/models/best_ncnn_model', [
            'models/best_ncnn_model/model.ncnn.param',
            'models/best_ncnn_model/model.ncnn.bin',
            'models/best_ncnn_model/metadata.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = line_follower.camera_node:main',
            'line_detection_node = line_follower.line_detection_node:main',
            'pid_node = line_follower.pid_node:main',
            'serial_node = line_follower.serial_node:main',
            'detection_node = line_follower.detection_node:main',
            'state_machine_node = line_follower.state_machine_node:main',
            'traffic_light_node = line_follower.traffic_light_node:main',
        ],
    },
)
