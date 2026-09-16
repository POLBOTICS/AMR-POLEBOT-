from setuptools import find_packages, setup

package_name = 'polebot_amr_csa'

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
    maintainer='riko',
    maintainer_email='rizkikomara321@gmail.com',
    description='CSA obstacle avoidance navigator',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csa_navigator = polebot_amr_csa.csa_navigator_revised:main',
        ],
    },
)
