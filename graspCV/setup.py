import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'graspCV'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude="test"),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhuang',
    maintainer_email='bin7.huang@zhangmen.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "grasp_cv_node = graspCV.grasp_service:main",
        ],
    },
)
