from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'pyqt_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*.png')),  # <-- 이 줄 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bako98',
    maintainer_email='kcjgh5132@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyqt_robot_2880x1620 = pyqt_robot.pyqt_robot_2880x1620:main',
            'pyqt_robot_1920x1080 = pyqt_robot.pyqt_robot_1920x1080:main',
        ],
    },
)
