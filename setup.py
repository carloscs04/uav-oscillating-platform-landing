from setuptools import setup, find_namespace_packages

setup(
    name="uav-oscillating-platform-landing",
    version="0.1.0",
    packages=find_namespace_packages(include=["src*"]),
    python_requires=">=3.12, <3.13",
)
