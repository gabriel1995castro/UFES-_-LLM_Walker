from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vwalker_hri'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'param'),  glob(os.path.join('param','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vwalker',
    maintainer_email='vwalker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teste = vwalker_hri.scan_physical2virtual:main'
        ],
    },
)
