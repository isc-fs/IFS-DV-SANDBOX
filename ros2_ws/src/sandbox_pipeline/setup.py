from setuptools import setup

package_name = "sandbox_pipeline"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ISC Driverless",
    maintainer_email="dv@iscracingteam.com",
    description="Baseline pipeline: one simple node per stage.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_baseline = sandbox_pipeline.odometry_baseline:main",
            "perception_baseline = sandbox_pipeline.perception_baseline:main",
            "slam_baseline = sandbox_pipeline.slam_baseline:main",
            "planning_baseline = sandbox_pipeline.planning_baseline:main",
            "control_baseline = sandbox_pipeline.control_baseline:main",
        ],
    },
)
