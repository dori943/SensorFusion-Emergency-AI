from setuptools import find_packages
from setuptools import setup

setup(
    name='sound_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('sound_interfaces', 'sound_interfaces.*')),
)
