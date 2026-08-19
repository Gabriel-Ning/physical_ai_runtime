from glob import glob

from setuptools import find_packages, setup

package_name = "rmi"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rmi"]),
        ("share/rmi", ["package.xml"]),
        (
            "share/rmi/config/embodiment_profiles",
            glob("config/embodiment_profiles/*.yaml"),
        ),
    ],
    install_requires=["pyyaml", "setuptools"],
    zip_safe=True,
    description="ROS Manipulation Interface (RMI) Python SDK for Physical AI Runtimes",
    license="Apache-2.0",
)
