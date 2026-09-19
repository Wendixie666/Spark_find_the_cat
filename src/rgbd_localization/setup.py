from setuptools import find_packages, setup


setup(
    name="rgbd_localization",
    version="0.0.1",
    packages=find_packages("src"),
    package_dir={"": "src"},
)
