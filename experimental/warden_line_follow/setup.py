import os
from glob import glob
from setuptools import setup

package_name = 'warden_line_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiwi',
    maintainer_email='maintainer@example.invalid',
    description='WARDEN line follower',
    license='Proprietary; see repository licensing note',
    entry_points={
        'console_scripts': [
            'line_follower = warden_line_follow.line_follower:main',
        ],
    },
)
