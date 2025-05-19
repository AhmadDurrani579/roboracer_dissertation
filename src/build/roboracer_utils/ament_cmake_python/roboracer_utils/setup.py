from setuptools import find_packages
from setuptools import setup

setup(
    name='roboracer_utils',
    version='0.0.0',
    packages=find_packages(
        include=('roboracer_utils', 'roboracer_utils.*')),
)
