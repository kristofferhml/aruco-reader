from setuptools import find_packages, setup

package_name = 'aruco_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kristoffer',
    maintainer_email='kristofferhumle@gmail.com',
    description='Generic ArUco marker detection from camera images.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = aruco_reader.main:main',
        ],
    },
)
