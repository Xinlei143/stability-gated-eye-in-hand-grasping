from glob import glob
from setuptools import find_packages, setup


package_name = "foam_grasp"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhang Yue",
    maintainer_email="yue24382@gtiit.edu.cn",
    description="Semantic segmentation, RGB-D fusion, target locking and safe Piper grasping.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "segmentation_node = foam_grasp.foam_segmentation_node:main",
            "depth_fusion_node = foam_grasp.foam_depth_fusion_node:main",
            "camera_to_base_node = foam_grasp.foam_camera_to_base_node:main",
            "target_latch_node = foam_grasp.foam_target_latch_node:main",
            "grasp_pose_preview_node = foam_grasp.foam_grasp_pose_preview_node:main",
            "grasp_ik_check = foam_grasp.foam_grasp_ik_check:main",
            "grasp_plan_check = foam_grasp.foam_grasp_plan_check:main",
            "grasp_cartesian_check = foam_grasp.foam_grasp_cartesian_check:main",
            "move_to_pregrasp = foam_grasp.foam_move_to_pregrasp:main",
            "move_to_observe = foam_grasp.foam_move_to_observe:main",
            "target_center = foam_grasp.foam_target_center_node:main",
            "object_grasp_sequence = foam_grasp.foam_cube_grasp_sequence:main",
            "cube_grasp_sequence = foam_grasp.foam_cube_grasp_sequence:main",
            "piper_gripper_safe_test = foam_grasp.piper_gripper_safe_test:main",
        ],
    },
)
