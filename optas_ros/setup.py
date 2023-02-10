import os
from glob import glob
from setuptools import setup

package_name = "optas_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name), glob("examples/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Christopher E. Mower",
    maintainer_email="christopher.mower@kcl.ac.uk",
    description="The optas_ros package.",
    license="Apache License Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "optas_controller_node = optas_ros.optas_controller_node:main",
            "zero_joint_state_publisher_node = optas_ros.zero_joint_state_publisher:main",
        ],
    },
)
