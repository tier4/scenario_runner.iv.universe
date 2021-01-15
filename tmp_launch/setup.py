from glob import glob

from setuptools import setup

package_name = 'tmp_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=[
        'launch',
        'launch_ros',
        'launch_xml',
        'numpy',
        'pyyaml',
        'setuptools',
        'termcolor',
    ],
    zip_safe=True,
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The planning_simulator_launcher package.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tmp = tmp_launch.tmp_launch:main',
        ],
    },
)
