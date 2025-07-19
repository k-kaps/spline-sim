from setuptools import find_packages, setup
from glob import glob
import os

package_name = "limo_spline"

setup(
    name=package_name,
    version="0.0.0",
    description="to simulate the spline package on the LIMO bot",
    maintainer="karan",
    maintainer_email="karan.kapoor@uwaterloo.ca",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/template_files", glob("limo_spline/template_files/*"))
    ],
    install_requires=["setuptools", "spline"],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'spline_path_publisher = limo_spline.spline_path_publisher:main',
            'obstacle_publisher = limo_spline.obstacle_publisher:main',
        ],
    }
)
