from setuptools import setup

setup(
    name="hercule_asserv",
    version="0.0.1",
    description="backstepping based control for unicycle robot",
    keywords="robot backstepping unicycle",
    packages=["hercule"],
    license="MIT",
    install_requires=["roboticstoolbox-python"],
    extras_require={
          'tests':  ["mypy", "pylint"]
    }
)
