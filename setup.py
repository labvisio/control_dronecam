import os
from glob import glob
from setuptools import find_packages, setup

package_name = "control_dronecam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
        (os.path.join("share", package_name), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mateus",
    maintainer_email="mateus.menines09@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imgpose_pub = control_dronecam.pose_aruco.imgpose_pub:main",
            "pub_tf =  control_dronecam.pub_tf.pub_tf:main",
            "init_parrot = control_dronecam.init_parrot.init_parrot:main",
            "control_parrot_keyboard = control_dronecam.control_parrot.control_parrot_keyboard:main",
            "keyboad_publish = control_dronecam.control_parrot.keyboad_publish:main",
            "control_parrot_aruco = control_dronecam.control_parrot.control_parrot_aruco:main",
        ],
    },
)
