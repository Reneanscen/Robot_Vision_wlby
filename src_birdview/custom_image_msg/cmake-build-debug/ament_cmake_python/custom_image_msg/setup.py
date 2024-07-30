from setuptools import find_packages
from setuptools import setup

setup(
    name='custom_image_msg',
    version='0.0.0',
    packages=find_packages(
        include=('custom_image_msg', 'custom_image_msg.*')),
)
