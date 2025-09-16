from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_orb_slam3',
    version='1.0.0',
    packages=find_packages(
        include=('ros2_orb_slam3', 'ros2_orb_slam3.*')),
)
