from setuptools import find_packages, setup

setup(
    name="mzj_system",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mzj_system"]),
        ("share/mzj_system", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MZJ maintainers",
    maintainer_email="maintainer@example.com",
    description="Command arbitration and freshness monitoring",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "system_node = mzj_system.node:main",
    ]},
)
