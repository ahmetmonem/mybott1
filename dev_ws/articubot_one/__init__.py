from setuptools import setup

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch_sim.launch.py',
            # أضف باقي ملفات launch التي تريدها
        ]),
        ('share/' + package_name + '/config', [
            'config/ball_tracker_params_robot.yaml',
            'config/ball_tracker_params_sim.yaml',
            # أضف باقي ملفات config التي تريدها
        ]),
        ('share/' + package_name + '/description', [
            'description/camera.xacro',
            'description/depth_camera.xacro',
            # أضف باقي ملفات description
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/empty.world',
            'worlds/fire_world.world',
            'worlds/obstacles.world',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Fire-fighting robot simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detector = articubot_one.fire_detector:main',
            'water_pump_controller = articubot_one.water_pump_controller:main',
        ],
    },
)
