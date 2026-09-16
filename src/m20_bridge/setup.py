from setuptools import find_packages, setup

package_name = 'm20_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    description='M20/M20 Pro basic_server bridge for rs_follow',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = m20_bridge.bridge_node:main',
            'fake_m20_server = m20_bridge.fake_m20_server:main',
        ],
    },
)
