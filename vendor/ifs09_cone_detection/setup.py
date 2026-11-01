from setuptools import find_packages, setup

package_name = "ifs09_cone_detection"

setup(
    name=package_name,
    version="0.0.0",
    # The Python package keeps its real name (cone_detection), so the code is
    # byte-for-byte the pipeline's and its imports work unchanged.
    packages=find_packages(include=["cone_detection", "cone_detection.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="ISC Driverless",
    maintainer_email="dv@iscracingteam.com",
    description="IFS09-DV-PIPELINE cone detection algorithm (vendored, no ROS node).",
    license="Apache-2.0",
)
