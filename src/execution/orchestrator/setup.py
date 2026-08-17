from glob import glob

from setuptools import find_packages, setup

package_name = "orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/trees", glob("trees/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description="RMI-backed orchestration control plane for Physical AI applications.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "orchestrator = orchestrator.cli:main",
        ],
    },
)
