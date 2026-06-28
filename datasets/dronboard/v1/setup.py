from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        "dr_onboard_autonomy",
        "dr_onboard_autonomy.message",
        "dr_onboard_autonomy.message.handlers",
        "dr_onboard_autonomy.message.senders",
        "dr_onboard_autonomy.states",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
