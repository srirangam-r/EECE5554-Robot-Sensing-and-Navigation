from setuptools import find_packages, setup

package_name = 'pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srirangamrh',
    maintainer_email='srirangam.r@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = pub_sub.pub:main',
            'subscriber = pub_sub.sub:main',
        ],
    },
)
