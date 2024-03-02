import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
package_name = 'reachy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='ggabriel9925@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_motion = reachy.move_to_point:main',
        ],
    },
)
