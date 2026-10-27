from setuptools import find_packages, setup

setup(
    name="mzj_localization",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mzj_localization"]),
        ("share/mzj_localization", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MZJ maintainers",
    maintainer_email="maintainer@example.com",
    description="Ideal odometry adapter for simulation; not a sensor fusion estimator",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "localization_node = mzj_localization.node:main",
    ]},
)
