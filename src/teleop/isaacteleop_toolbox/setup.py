from glob import glob

from setuptools import find_packages, setup

package_name = "isaacteleop_toolbox"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "LICENSE", "CHANGELOG.rst"],
        ),
        (f"share/{package_name}/configs", glob("configs/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.12",
    zip_safe=False,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description="ROS 2 teleoperation source framework built on IsaacTeleop.",
    license="Apache-2.0",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering",
    ],
    entry_points={
        "console_scripts": [
            "isaacteleop-cloudxr-setup = isaacteleop_toolbox.cloudxr_setup:main",
            "quest3_bimanual_target = isaacteleop_toolbox.nodes.quest3_bimanual_target_node:main",
            "quest3_bimanual_absolute_target = isaacteleop_toolbox.nodes.quest3_bimanual_absolute_node:main",
        ],
    },
)
