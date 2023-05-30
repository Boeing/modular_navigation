import os
from glob import glob
from setuptools import setup

package_name = 'autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    py_modules=[
        'fake_localisation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    author='Boeing',
    maintainer='Phillip Haeusler',
    maintainer_email='Phillip.Haeusler@boeing.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Fake localisation publisher to avoid using cartographer for testing purposes',
    license='Boeing Proprietary',
    entry_points={
        'console_scripts': [
            'fake_localisation = fake_localisation:main'
        ],
    },
)
