from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'omni_pendulum_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes', 'limo'), glob('meshes/limo/*_lite.dae')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='2D Inverted Pendulum Omnidirectional Robot Simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = omni_pendulum_sim.balance_controller:main',
            'disturbance = omni_pendulum_sim.disturbance:main',
            'keyboard_nudge = omni_pendulum_sim.keyboard_nudge:main',
        ],
    },
)
