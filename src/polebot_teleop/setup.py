from setuptools import find_packages, setup

package_name = 'polebot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MiraeNK',
    maintainer_email='polman@polman-bandung.ac.id',
    description='Keyboard and joystick teleop for AMR-POLEBOT',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = polebot_teleop.keyboard_teleop:main',
        ],
    },
)
