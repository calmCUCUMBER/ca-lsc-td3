import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'ca_lsc_transition'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weicheng',
    maintainer_email='weicheng@todo.todo',
    description='CA-LSC-TD3 transition experiment and telemetry tools.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'transition_experiment = '
            'ca_lsc_transition.experiment_node:main',
            'evaluate_transition = '
            'ca_lsc_transition.evaluate_run:main',
        ],
    },
)
