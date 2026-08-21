from glob import glob
from setuptools import find_packages, setup


package_name = "foam_grasp_sim"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/models/table", ["models/table/model.sdf"]),
        ("share/" + package_name + "/models/cube", ["models/cube/model.sdf"]),
        (
            "share/" + package_name + "/models/cylinder",
            ["models/cylinder/model.sdf"],
        ),
        ("share/" + package_name + "/models/sphere", ["models/sphere/model.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhang Yue",
    maintainer_email="yue24382@gtiit.edu.cn",
    description="Gazebo Classic simulation composition for foam grasping.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "static_target_source_node = "
            "foam_grasp_sim.static_target_source_node:main",
            "target_motion_node = foam_grasp_sim.target_motion_node:main",
            "simulated_perception_node = "
            "foam_grasp_sim.simulated_perception_node:main",
            "method_policy_node = "
            "foam_grasp_sim.method_policy_node:main",
            "metrics_logger_node = "
            "foam_grasp_sim.metrics_logger_node:main",
        ],
    },
)
