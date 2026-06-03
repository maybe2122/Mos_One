from setuptools import find_packages, setup


setup(
    name="stackforce_mos",
    version="0.1.0",
    description="StackForce SimReady closed-chain USD Isaac Lab export for StackForce-Mos20262ClosedUsd-ClosedUsd-v0",
    packages=find_packages(),
    include_package_data=True,
    install_requires=["psutil"],
    python_requires=">=3.10",
)
