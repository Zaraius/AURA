from setuptools import setup

package_name = 'aura'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/test_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/auto_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_launch.py']),
        ('share/' + package_name + '/launch', ['launch/auto_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/my_world.wbt']),
        ('share/' + package_name + '/resource', ['resource/my_robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Aura minimal package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'subscribe = aura.drive_subscribe:main',
            'teleop = aura.teleop_ackermann:main',
            'ackermann_driver = aura.ackermann_driver:main',
            'state_machine = aura.state_machine_node:main',
            'auto = aura.autonomous_node:main',
            'controller = aura.teleop_auto_controller:main',
            'encoder = aura.rotary_encoder:main',
        ],
    },
)
