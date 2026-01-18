from setuptools import find_packages
from setuptools import setup

setup(
    name='arm_moveit_task',
    version='0.1.0',
    packages=find_packages(
        include=('arm_moveit_task', 'arm_moveit_task.*')),
)
