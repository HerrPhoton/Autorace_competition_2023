import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'competition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A. Leisle',
    maintainer_email='a.leisle@g.nsu.ru',
    description='Package controlling the robot at autorace 2023',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lane_detect = competition.lane_detect:main",
            "lane_follow = competition.lane_follow:main",
        ],
    },
)
