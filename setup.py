import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'precision_jig'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hubis_jhjung',
    maintainer_email='scarlet777735@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'amr_vision_check_gui    =    precision_jig.amr_vision_check_gui:main',
            'amr_vision_check        =    precision_jig.amr_vision_check:main',

        ],
    },
)
