from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srirangamrh',
    maintainer_email='srirangam.r@northeastern.edu',
    description='GPS driver for UB-353',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = gps_driver.driver:main',
        ],
    },
)
