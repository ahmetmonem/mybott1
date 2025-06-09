from setuptools import find_packages
from setuptools import setup

setup(
    name='articubot_one',
    version='0.0.0',
    packages=find_packages(
        include=('articubot_one', 'articubot_one.*')),
)
