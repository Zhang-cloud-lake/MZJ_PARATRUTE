from setuptools import find_packages, setup

setup(
    name="mzj_mcu_bridge",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mzj_mcu_bridge"]),
        ("share/mzj_mcu_bridge", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MZJ maintainers",
    maintainer_email="maintainer@example.com",
    description="Example framed serial protocol with dry-run output",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mcu_bridge = mzj_mcu_bridge.node:main",
    ]},
)
