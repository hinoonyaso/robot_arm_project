from setuptools import find_packages, setup

package_name = "pick_place_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pick_place.launch.py"]),
        ("share/" + package_name + "/config", ["config/pick_place_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Pick and place state machine with MoveIt2 integration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place_state_machine = pick_place_task.pick_place_state_machine:main",
        ],
    },
)
