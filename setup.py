from setuptools import setup
import os
from glob import glob

package_name = 'openarm_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['schemas/*.json']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'schemas'),
            glob('openarm_api/schemas/*.json')),
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*')),
    ],
    install_requires=['setuptools', 'jsonschema'],
    extras_require={
        'http': ['fastapi', 'uvicorn', 'websockets'],
    },
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='JSON gateway / upper-computer interface for OpenArm skills.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'json_bridge_node = openarm_api.json_bridge_node:main',
        ],
    },
)
