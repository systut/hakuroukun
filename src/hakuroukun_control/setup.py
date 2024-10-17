from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['hakuroukun_control'],
    package_dir={'': ''},
)

setup(**setup_args)