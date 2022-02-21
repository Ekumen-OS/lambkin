#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['lambkin'],
    package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'lambkin=lambkin.cli:main',
        ]
    }
)

setup(**setup_args)
