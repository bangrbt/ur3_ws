import os
from glob import glob
from setuptools import setup, find_packages

package_name = "ur3_llm_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name if os.path.exists("resource/" + package_name) else "package.xml"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools", "pyyaml", "requests"],
    zip_safe=True,
    maintainer="Le Anh Tuan Bang",
    maintainer_email="23020723@vnu.edu.vn",
    description="Package dieu khien robot UR3/UR3e bang LLM va Skill-based Planning",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_interactive_node = ur3_llm_control.llm_interactive_node:main",
            "scene_spawner = ur3_llm_control.scene_spawner:main",
        ],
    },
)
