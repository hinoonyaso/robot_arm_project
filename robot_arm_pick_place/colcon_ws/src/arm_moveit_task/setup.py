from setuptools import setup

package_name = "arm_moveit_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dev",
    maintainer_email="dev@example.com",
    description="Pick and place task node with attach/detach and stress testing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place_task = arm_moveit_task.pick_place_task:main",
        ],
    },
)
