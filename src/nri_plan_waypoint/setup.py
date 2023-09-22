import os
from setuptools import setup

package_name = 'nri_plan_waypoint'

share_dir = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artlab',
    maintainer_email='eehoa12381@gmai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['nri_plan_waypoint = nri_plan_waypoint.nri_plan_waypoint_node:main'
        ],
    },
)
