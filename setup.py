import setuptools

with open("README.md", "r") as readme:
    long_description = readme.read()

setuptools.setup(
    name="jamz_drone_autopilot",
    version="0.0.1",
    author="Logan Rodie",
    author_email="lrodi045@uottawa.ca",
    description="JAMZ Autonomous Delivery Autopilot Package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Abstract-9/JAMZ_Drone",
    packages=setuptools.find_packages,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent"
        "License :: None :: Proprietary"
    ],
    install_requires=[
        "requests~=2.24.0",
        "dronekit~=2.9.2",
        "aiocoap~=0.3",
        "pymavlink~=2.4.9"
    ],
    entry_points={
        'console_scripts': [
            'jamz_autopilot = jamz_drone:initialize'
        ]
    },
    python_requires='>=3.6'
)