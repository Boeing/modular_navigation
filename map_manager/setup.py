# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'map_manager'

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages[package_name]#, package_name+'/dxf', package_name+'/http_utils'],
    #packages=find_packages(where='map_manager'),
    package_dir={'': 'map_manager'}),
    #py_modules=['map_manager.map_manager_node']
)
setup(**setup_args)
