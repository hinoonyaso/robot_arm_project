from setuptools import find_packages, setup

package_name = "pick_place_moveit_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/moveit.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/moveit.rviz",
            "config/robot.urdf.xacro",
            "config/simple_arm.srdf",
            "config/kinematics.yaml",
            "config/ompl_planning.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="MoveIt2 configuration placeholders for pick and place.",
    license="Apache-2.0",
)
