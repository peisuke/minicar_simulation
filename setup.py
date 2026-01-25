from setuptools import find_packages, setup

package_name = 'minicar_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/road_env_minicar.launch.py',
            'launch/road_env_ackermann.launch.py',
        ]),
        ('share/' + package_name + '/description/launch', ['description/launch/robot_state_publisher.launch.py']),
        ('share/' + package_name + '/description/urdf', [
            'description/urdf/minicar_diff_gazebo.xacro',
            'description/urdf/minicar_ackermann_gazebo.xacro',
        ]),
        ('share/' + package_name + '/description/config', [
            'description/config/minicar_diff_controller.yaml',
            'description/config/minicar_ackermann_controller.yaml',
        ]),
        ('share/' + package_name + '/worlds', ['worlds/road_env.world']),
        # Note: models/road_env/* are generated at runtime to /tmp/minicar_simulation/models/
        # Course generation scripts
        ('share/' + package_name + '/scripts', ['scripts/generate_course.py']),
        ('share/' + package_name + '/scripts/create_course', [
            'scripts/create_course/__init__.py',
            'scripts/create_course/models.py',
            'scripts/create_course/generator.py',
            'scripts/create_course/exporter.py',
            'scripts/create_course/ellipse.py',
            'scripts/create_course/spline.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='peisuke.com@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reset_robot = minicar_simulation.reset_robot_node:main',
        ],
    },
)
