from setuptools import setup

package_name = "sandbox_tools"

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
    description="Sandbox helpers and tool nodes (ground truth, mock sim, viz, referee).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gt_cones = sandbox_tools.gt_cones:main",
            "gt_odom = sandbox_tools.gt_odom:main",
            "lidar_skewer = sandbox_tools.lidar_skewer:main",
            "visualizer = sandbox_tools.visualizer:main",
            "monitor = sandbox_tools.monitor:main",
            "mock_sim = sandbox_tools.mock_sim:main",
            "teleop = sandbox_tools.teleop:main",
        ],
    },
)
