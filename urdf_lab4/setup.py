from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'urdf_lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('urdf_lab4/*.yaml')),
        (os.path.join('share', package_name), glob('urdf_lab4/*.json')),
        #(os.path.join('share', package_name), glob('urdf_lab4/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lena',
    maintainer_email='01149451@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #'state_publisher = urdf_lab4.state_publisher:main',
        #'nonkdl_dkin = urdf_lab4.nonkdl_dkin:main',
        #'kdl_dkin = urdf_lab4.kdl_dkin:main',
        'service = urdf_lab4.jint_control_srv:main',
        'client = urdf_lab4.jint:main',
        'publ = urdf_lab4.state_publisher:main', 
        'service2 = urdf_lab4.oint_control_srv:main',
        'client2 = urdf_lab4.oint:main',
        
        ],
    },
)
