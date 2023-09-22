from setuptools import setup

package_name = 'agent_gps_test'

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
        'console_scripts': ['agent_gps_test = agent_gps_test.agent_gps_test_node:main'
        ],
    },
)
