"""Setup for meca500_demos package."""
from setuptools import find_packages, setup

package_name = 'meca500_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/planningscene.launch.xml']),
        ('share/' + package_name + '/config', ['config/objects.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishika2024',
    maintainer_email='berarishika@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_node = meca500_demos.scene_node:main',
        ],
    },
)
