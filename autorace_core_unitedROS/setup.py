import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_core_unitedROS'

data_files = []

start_point = os.path.join('model')
for root, dirs, files in os.walk(start_point):
    root_files = [os.path.join(root, i) for i in files]
    data_files.append((os.path.join('share', package_name, root), root_files))

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append((os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.pt'))))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools==58.2.0', 'dill'],
    zip_safe=True,
    maintainer='A. Leisle',
    maintainer_email='a.leisle@g.nsu.ru',
    description='Package controlling the robot at autorace 2023',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lane_detect = autorace_core_unitedROS.lane_detect:main",
            "lane_follow = autorace_core_unitedROS.lane_follow:main",
            "avoid_obstacles = autorace_core_unitedROS.avoid_obstacles:main",
            "sign_detection = autorace_core_unitedROS.sign_detection:main",
            "traffic_light = autorace_core_unitedROS.traffic_light:main",
            "intersection = autorace_core_unitedROS.intersection:main",
            "pedestrian_crossing = autorace_core_unitedROS.pedestrian_crossing:main",
            "parking = autorace_core_unitedROS.parking:main",
            "robot_rotator = autorace_core_unitedROS.robot_rotator:main"

        ],
    },
)
