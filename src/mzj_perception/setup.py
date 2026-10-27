from setuptools import find_packages, setup

setup(
    name="mzj_perception",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mzj_perception"]),
        ("share/mzj_perception", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MZJ maintainers",
    maintainer_email="maintainer@example.com",
    description="Image-space perception with color demo and optional local YOLO weights",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "perception_node = mzj_perception.node:main",
        "demo_camera = mzj_perception.demo_camera:main",
    ]},
)
