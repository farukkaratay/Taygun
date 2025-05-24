from setuptools import find_packages
from setuptools import setup

setup(
    name='px4_offboard_control',
    version='0.1.0',
    packages=find_packages(
        include=('px4_offboard_control', 'px4_offboard_control.*')),
)
