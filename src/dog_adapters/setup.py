from glob import glob
from setuptools import find_packages, setup

package_name = 'dog_adapters'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@example.com',
    description='Shared command safety gate and Twist/Unitree robot adapters',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'twist_adapter = dog_adapters.twist_adapter:main',
        'unitree_adapter = dog_adapters.unitree_adapter:main',
    ]},
)
