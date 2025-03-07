from setuptools import find_packages, setup

package_name = 'srv_cli'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = srv_cli.srv:main',
            'client = srv_cli.cli:main',
        ],
    },
)
