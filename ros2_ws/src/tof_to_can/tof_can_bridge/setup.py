from setuptools import setup

package_name = 'tof_can_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='jeongmin.choi',
    maintainer_email='jeongmin.choi@wattrobotics.ai',
    description='ToF-over-CAN bridge node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = tof_can_bridge.bridge_node:main',
        ],
    },
)
