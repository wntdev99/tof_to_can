from setuptools import setup

package_name = 'tof_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/eval_params.example.yaml']),
        ('share/' + package_name + '/launch', ['launch/eval.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='jeongmin.choi',
    maintainer_email='jeongmin.choi@wattrobotics.ai',
    description='PointCloud2 evaluation node for ToF sensor comparison',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'eval_node = tof_eval.eval_node:main',
        ],
    },
)
