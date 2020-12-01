# -*- coding: utf-8 -*-
"""
Setup file of the `panda_autograsp` python package. This setup file
was based upon the gqcnn setup.py file. This file makes sure that all
the dependencies of the submodules are installed. This can be done in
two ways:

    - METHOD1: You can use the `add_submods_requirements` function
    to add the requirements of each submodule to the setup.py requirements
    list.
    NOTE: Advised for ROS packages as the submodules themselves don't get installed.
    - METHOD2: You can use the `install_submods` to run the setup.py of each
    submodule as a subprocess. To do this set install_dependencies to true.

Author
------
Rick Staa
"""
# Future Imports
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# Standard library imports
import logging
import os
from setuptools import setup, find_packages
from setuptools.command.develop import develop
from setuptools.command.install import install
import subprocess
import sys
import re
import shutil
from collections import OrderedDict

# Set setup.py submodule install method
SUB_MOD_REQ_INSTALL_METHOD = 1
IS_ROS_PACKAGE = True  # If true the submodules themself will not be installed

# General setup.py parameters
TF_MAX_VERSION = "1.15.0"

# Pre-install requirements (Installed before the install class is run)
install_requirements = ["scipy", "numpy==1.17.4", "cython"]

# Package requirements
requirements = [
    "tensorflow==1.15.0",
    "pyglet",
    "scipy",
    "scikit-learn",
    "ruamel.yaml",
    "multiprocess",
    "setproctitle",
    "joblib",
    "colorlog",
    "autolab-core",
    "visualization",
    "opencv-python",
    "opencv-contrib-python",
    "scikit-image",
    "gputil",
    "pylibfreenect2",
    "tensorflow",
    "tensorflow-estimator",
    "pyquaternion",
    "keras",
    "ipython",
    "scikit-video",
    "ffmpeg-python",
    "Pillow",
    "docutils",
    "pyserial",
    "cython",
    "psutil",
    "matplotlib",
    "numpy",
    "cycler",
    "rospkg",
    "defusedxml",
    "PySide2",
    "netifaces",
]

# Set up logger.
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
logger.addHandler(handler)


#################################################
# Setup functions################################
#################################################
def get_tf_dep():
    """Check whether or not the Nvidia driver and GPUs are available and add the
    corresponding Tensorflow dependency."""

    # Find the right tensorflow version to install
    tf_dep = "tensorflow<={}".format(TF_MAX_VERSION)
    try:
        gpus = (
            subprocess.check_output(
                ["nvidia-smi", "--query-gpu=gpu_name", "--format=csv"]
            )
            .decode()
            .strip()
            .split("\n")[1:]
        )
        if len(gpus) > 0:
            tf_dep = "tensorflow-gpu<={}".format(TF_MAX_VERSION)
        else:
            no_device_msg = (
                "Found Nvidia device driver but no"
                " devices...installing Tensorflow for CPU."
            )
            logger.warning(no_device_msg)
    except OSError:
        no_driver_msg = (
            "Could not find Nvidia device driver...installing" " Tensorflow for CPU."
        )
        logger.warning(no_driver_msg)
    return tf_dep


def get_git_submods():
    """Returns a list containing the git submodules.

    Returns
    --------
    :py:obj:`list` of :py:obj:`string`
        List containing the names of the submodules
    """

    # Find git submodules
    sub_mod_bash_command = (
        "git config --file "
        + os.getcwd()
        + "/.gitmodules --get-regexp path | awk '{ print $2 }'"
    )
    bash_output = subprocess.check_output(["bash", "-c", sub_mod_bash_command])
    sub_mods = [x for x in bash_output.decode("utf-8").split("\n") if x != ""]

    # Return a list with submodule
    return sub_mods


def add_submods_requirements(requirements):
    """Adds the submodule requirements to the requirements list.

    Returns
    -------
    :py:obj:`list` of :py:obj:`string`
        List containing all the requirements.
    :py:obj:`list` of :py:obj:`string`
        List containing the names of the submodules.
    :py:obj:`list` of :py:obj:`string`
        List containing the python egg (package) names of the submodules.
    """

    # Get git submodule names
    sub_mods = get_git_submods()

    # Initialize list to put egg (package) names in
    sub_mods_egg_names = []

    # Append submodule requirements to the requirements list
    for sub_mod in sub_mods:
        submod_path = os.getcwd() + "/" + sub_mod
        submod_setup_path = submod_path + "/setup.py"
        if os.path.exists(submod_setup_path):

            # Generate requires.txt file for the submodule using the setuptools.egg
            # info module
            try:

                # Run the egg_info command
                out, _ = subprocess.Popen(
                    [sys.executable, submod_setup_path, "egg_info"],
                    stderr=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    cwd=submod_path,
                ).communicate()

                # Retrieve egg-name and fodler path
                egg_name = re.search(
                    r"\nwriting.(.*?).egg-info", str(out, "utf-8")
                ).group(1)
                sub_mods_egg_names.append(egg_name)  # Append egg (package) name
                egg_folder_path = (
                    os.getcwd() + "/" + sub_mod + "/" + egg_name + ".egg-info"
                )
                # Open egg_info generated requires.txt file
                with open(egg_folder_path + "/requires.txt") as file:

                    # Read requires file up till empty line
                    for line in file:
                        if line.strip() == "":

                            # Try to remove tree; if failed show an error using try
                            # ...except on screen
                            try:
                                # Remove egg file
                                shutil.rmtree(egg_folder_path)
                            except OSError as e:
                                print("Error: %s - %s." % (e.filename, e.strerror))
                            break
                        else:

                            # Append submodule requirement to package requirements
                            requirements.append(line.strip())
            except Exception as e:
                logger.warning(
                    "Submodule dependencies could not be imported. " + str(e)
                )

    # Remove duplicate items
    requirements = list(OrderedDict.fromkeys(requirements))

    # Return submodule requirements and submodules
    return requirements, sub_mods, sub_mods_egg_names


def install_submods(sub_mods, mode="install", install_dependencies=True):
    """Installs the submodules

    Parameters
    ----------
    mode : str, optional
        The installation mode to use, by default "install"
    install_dependencies : bool, optional
        Install submodule dependencies, by default True
    """

    # Normal installation
    # Install submodules in non development mode
    if mode.lower() == "install":
        for sub_mod in sub_mods:
            submod_path = os.getcwd() + "/" + sub_mod
            submod_setup_path = submod_path + "/setup.py"
            if os.path.exists(submod_setup_path):
                if install_dependencies:  # Install submodule dependencies
                    subprocess.Popen(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "install",
                            os.getcwd() + "/" + sub_mod + "/",
                        ],
                        cwd=submod_path,
                    ).communicate()
                else:  # Do not install submodule dependencies
                    subprocess.Popen(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "install",
                            "--no-dependencies",
                            os.getcwd() + "/" + sub_mod + "/",
                        ],
                        cwd=submod_path,
                    ).communicate()

    # Development mode installation
    # Install submodules in development mode
    if mode.lower() == "develop":
        for sub_mod in sub_mods:
            submod_path = os.getcwd() + "/" + sub_mod
            submod_setup_path = submod_path + "/setup.py"
            if os.path.exists(submod_setup_path):
                if install_dependencies:  # Install submodule dependencies
                    subprocess.Popen(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "install",
                            os.getcwd() + "/" + sub_mod + "/",
                        ],
                        cwd=submod_path,
                    ).communicate()
                else:  # Do not install submodule dependencies
                    subprocess.Popen(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "install",
                            "--no-dependencies",
                            os.getcwd() + "/" + sub_mod + "/",
                        ],
                        cwd=submod_path,
                    ).communicate()


#################################################
# Setup classes##################################
#################################################
class DevelopCmd(develop):
    """Overload :py:class:`setuptools.command.develop.develop` class."""

    # Add extra user arguments
    user_options_custom = [("docker", None, "installing in Docker")]
    user_options = getattr(develop, "user_options", []) + user_options_custom

    def initialize_options(self):
        """Initialize extra argument options."""
        develop.initialize_options(self)

        # Initialize options.
        self.docker = False

    def finalize_options(self):
        """Set extra install arguments."""
        develop.finalize_options(self)

    def run(self):
        """Overload the :py:meth:`setuptools.command.develop.develop.run` method."""

        # Install Tensorflow dependency.
        if not self.docker:
            tf_dep = get_tf_dep()
            subprocess.Popen([sys.executable, "-m", "pip", "install", tf_dep]).wait()
        else:
            # If we're using Docker, this will already have been installed
            # explicitly through the correct `{cpu/gpu}_requirements.txt`;
            # there is no way to check for CUDA/GPUs at Docker build time
            # because there is no easy way to set the Nvidia runtime.
            # TODO(vsatish): Figure out why this isn't printed.
            skip_tf_msg = (
                "Omitting Tensorflow dependency because of Docker" " installation."
            )
            logger.warning(skip_tf_msg)

        # Install submodules
        global sub_mods
        if SUB_MOD_REQ_INSTALL_METHOD == 2:
            install_submods(sub_mods, "develop", install_dependencies=True)

        # Run parent run method
        develop.run(self)  # Disabled because no top level python packages are present

        # Print success message
        if SUB_MOD_REQ_INSTALL_METHOD == 2:
            print(
                "All the python submodules in this catkin_ws and their requirements "
                "have been installed successfully."
            )
        else:
            print(
                "All the requirements of the python submodules in this catkin_ws have "
                "been installed successfully. The python submodules themselves are "
                "assumed to be added to the PYTHONPATH by catkin."
            )


class InstallCmd(install, object):
    """Overload :py:class:`setuptools.command.install.install` class"""

    # Add extra user arguments
    user_options_custom = [("docker", None, "installing in Docker")]
    user_options = getattr(install, "user_options", []) + user_options_custom

    def initialize_options(self):
        """Initialize extra argument options."""
        install.initialize_options(self)

        # Initialize options.
        self.docker = False

    def finalize_options(self):
        """Set extra argument options."""
        install.finalize_options(self)

    def run(self):
        """Overload the :py:meth:`setuptools.command.install.install.run` method."""

        # Install Tensorflow dependency.
        if not self.docker:
            tf_dep = get_tf_dep()
            subprocess.Popen([sys.executable, "-m", "pip", "install", tf_dep]).wait()
        else:
            # If we're using Docker, this will already have been installed
            # explicitly through the correct `{cpu/gpu}_requirements.txt`;
            # there is no way to check for CUDA/GPUs at Docker build time
            # because there is no easy way to set the Nvidia runtime.
            # TODO (vsatish): Figure out why this isn't printed.
            skip_tf_msg = (
                "Omitting Tensorflow dependency because of Docker" " installation."
            )
            logger.warning(skip_tf_msg)

        # Install submodules
        global sub_mods
        if SUB_MOD_REQ_INSTALL_METHOD == 2:
            install_submods(sub_mods, "install", install_dependencies=True)

        # Run parent run method
        install.run(self)  # Disabled because no top level python packages are present

        # Print success message
        if SUB_MOD_REQ_INSTALL_METHOD == 2:
            print(
                "All the python submodules in this catkin_ws and their requirements "
                "have been installed successfully."
            )
        else:
            print(
                "All the requirements of the python submodules in this catkin_ws have "
                "been installed successfully. The python submodules themselves are "
                "assumed to be added to the PYTHONPATH by catkin."
            )


#################################################
# Setup script###################################
#################################################

# Get current package version
__version__ = re.sub(
    r"[^\d.]",
    "",
    open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "panda_autograsp/version.py"
        )
    ).read(),
)

# Install start messages
if SUB_MOD_REQ_INSTALL_METHOD == 2:
    print(
        "Install catkin workspace python submodules using method 2."
        "This method installs all python submodules present in the catkin workspace "
        "together with their python requirements. This method is not advised when "
        "you are working with ROS packages as catkin already adds theses submodules"
        "to the PYTHONPATH. Installing both the python packages themselves and their "
        "requirements can lead to conflicts. Please select install method 1 when "
        "working with ROS packages."
    )
else:
    print(
        "Install catkin workspace python submodules using method 1. "
        "This method only installs the python requirements of the submodules "
        "while the submodules themselves are not installed. This is the advices "
        "method when you are working with ROS packages as catkin already adds these "
        "submodules to the PYTHONPATH."
    )

# Install pre-install requirements
for req in install_requirements:
    subprocess.Popen([sys.executable, "-m", "pip", "install", req]).communicate()

# Check install method
if SUB_MOD_REQ_INSTALL_METHOD == 2:

    # Get git submodule names
    sub_mods = get_git_submods()
else:
    # Add submodule requirements to the requirements list
    print("Retrieving submodule requirements...")
    requirements, sub_mods, _ = add_submods_requirements(requirements)

    # Remove submodule packages from requirements
    requirements = [
        j for j in requirements if not any([i.replace("_", "-") in j for i in sub_mods])
    ]
    print("Submodule requirements retrieval successful.")

# Add panda_autograsp as a submodule
sub_mods.extend("panda_autograsp")

# Run python setup
setup(
    name="panda_autograsp_ws",
    version=__version__,
    description=("Project code for the panda_autograsp automatic grasping solution."),
    author="Rick Staa",
    author_email="rick.staa@outlook.com",
    license="Rick Staa Copyright",
    url="https://github.com/rickstaa/panda_autograsp",
    keywords="robotics grasping vision deep learning franka gqcnn gpd",
    classifiers=[
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering",
    ],
    packages=find_packages(),
    include_package_data=True,
    install_requires=requirements,
    extras_require={
        "docs": [
            "sphinx",
            "sphinxcontrib-napoleon",
            "sphinx_rtd_theme",
            "sphinx-navtree",
            "sphinx-autobuild",
            "sphinxcontrib.yt",
            "docutils",
            "doc8",
            # "breathe==4.13.1",
        ],
        "dev": ["pytest", "bumpversion", "pylint"],
    },
    cmdclass={"install": InstallCmd, "develop": DevelopCmd},
)
