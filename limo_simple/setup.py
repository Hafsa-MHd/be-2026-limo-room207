from glob import glob

from setuptools import find_packages, setup

package_name = "limo_simple"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BE LIMO",
    maintainer_email="student@univ-lorraine.fr",
    description="Room 207 perception + polar / lane follow control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "road_vision = limo_simple.road_vision:main",
            "lane_follow = limo_simple.lane_follow:main",
            "polar_follow = limo_simple.polar_follow:main",
            "spawn_parking = limo_simple.spawn_parking:main",
        ],
    },
)
