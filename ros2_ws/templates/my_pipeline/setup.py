from setuptools import setup

package_name = "my_pipeline"

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
    maintainer="me",
    maintainer_email="me@example.com",
    description="My own pipeline nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # name = module:function. After adding a new one, run sb-build again.
            "odometry = my_pipeline.odometry:main",
            "perception = my_pipeline.perception:main",
            "slam = my_pipeline.slam:main",
            "planning = my_pipeline.planning:main",
            "control = my_pipeline.control:main",
        ],
    },
)
