from setuptools import setup

package_name = 'optas_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher E. Mower',
    maintainer_email='christopher.mower@kcl.ac.uk',
    description='The optas_ros package.',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optas_node = optas_ros.optas_node:main'
        ],
    },
)
