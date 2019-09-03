# -*- coding: utf-8 -*-
"""
Setup file of the `panda_autograsp` python package. This setup file was based upon the gqcnn setup.py file.

Author
------
Rick Staa
"""

## Future Imports ##
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

## Standard library imports ##
import logging
import os
from setuptools import setup, find_packages
from setuptools.command.develop import develop
from setuptools.command.install import install
import subprocess
import sys
import re
import shutil

## General setup.py parameters ##
TF_MAX_VERSION = "1.13.1"

## Package requirements ##
if (sys.version_info > (3, 0)): # Py 2 requirements
    requirements = ['rospkg', 'defusedxml', 'empy', 'catkin-pkg', 'pyQt5', 'PySide2', 'pyassimp']
else: # Py 3 requirements
    requirements = ['rospkg', 'defusedxml', 'empy', 'catkin-pkg', 'PySide2', 'pyassimp']

## Set up logger. ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
logger.addHandler(handler)

#################################################
## Setup functions ##############################
#################################################
def add_submods_requirements(requirements):
    """Adds the submodule requirements to the requirements list.

    Returns
    -------
    :obj:`list` of :obj:`string`
        List containing all the requirements.
    :obj:`list` of :obj:`string`
        List containing the names of the submodules.
    :obj:`list` of :obj:`string`
        List containing the python egg (package) names of the submodules.
    """

    ## Find git submodules ##
    sub_mod_bash_command = "git config --file " + \
        os.getcwd()+"/.gitmodules --get-regexp path | awk '{ print $2 }'"
    bash_output = subprocess.check_output(['bash', '-c', sub_mod_bash_command])
    sub_mods = [x for x in bash_output.decode("utf-8").split("\n") if x != '']
    sub_mods_egg_names = [] # Initialize list to put egg (package) names in

    ## Append submodule requirements to the requirements list ##
    for sub_mod in sub_mods:
       submod_setup_path = os.getcwd()+"/"+sub_mod+"/setup.py"
       if os.path.exists(submod_setup_path):

            # Generate requires.txt file for the submodule using the setuptools.egg_info module
            try:
                # Run the egg_info command #
                proc = subprocess.Popen(
                    [sys.executable, submod_setup_path, "egg_info"], stderr=subprocess.PIPE, stdout=subprocess.PIPE)

                # Retrieve egg-name #
                out, _ = proc.communicate()
                egg_name = re.search(r'\nwriting.(.*?).egg-info', str(out,"utf-8")).group(1)
                sub_mods_egg_names.append(egg_name) # Append egg (package) name

                # Open egg_info generated requires.txt file
                with open(os.getcwd()+"/"+egg_name+".egg-info/requires.txt") as file:

                    # Read requires file up till empty line #
                    for line in file:
                        if line.strip() == "":

                            # Try to remove tree; if failed show an error using try...except on screen #
                            try:
                                shutil.rmtree(os.getcwd()+"/"+egg_name+".egg-info")
                            except OSError as e:
                                print("Error: %s - %s." %
                                      (e.filename, e.strerror))
                            break
                        # Append submodule requirement to package requirements #
                        requirements.append(line.strip())
            except Exception as e:
                logger.warning(
                    "Submodule dependencies could not be imported. "+str(e))

    ## Remove duplicate items ##
    requirements = list(set(requirements))

    ## Return submodule requirements and submodules ##
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

    ## Normal installation ##
    # Install submodules in non development mode
    if mode.lower() == "install":
        for sub_mod in sub_mods:
           submod_setup_path = os.getcwd()+"/"+sub_mod+"/setup.py"
           if os.path.exists(submod_setup_path):
                if install_dependencies:  # Install submodule dependencies
                    subprocess.call(
                        [sys.executable, "-m", "pip", "install", os.getcwd()+"/"+sub_mod+"/"])
                else:  # Do not install submodule dependencies
                    subprocess.call([sys.executable, "-m", "pip", "install",
                                     "--no-dependencies", os.getcwd()+"/"+sub_mod+"/"])

    ## Development mode installation ##
    # Install submodules in development mode
    if mode.lower() == "develop":
        for sub_mod in sub_mods:
           submod_setup_path = os.getcwd()+"/"+sub_mod+"/setup.py"
           if os.path.exists(submod_setup_path):
                if install_dependencies:  # Install submodule dependencies
                    # subprocess.call(
                    #     [sys.executable, "-m", "pip", "install", "-e", os.getcwd()+"/"+sub_mod+"/"])
                    subprocess.call(
                        [sys.executable, "-m", "pip", "install", os.getcwd()+"/"+sub_mod+"/"])
                else:  # Do not install submodule dependencies
                    # subprocess.call([sys.executable, "-m", "pip", "install",
                    #                  "--no-dependencies", "-e", os.getcwd()+"/"+sub_mod+"/"])
                    subprocess.call([sys.executable, "-m", "pip", "install",
                                     "--no-dependencies", os.getcwd()+"/"+sub_mod+"/"])

#################################################
## Setup classes ################################
#################################################
class DevelopCmd(develop):
    """Overload :py:class:`setuptools.command.develop.develop` class."""

    ## Add extra user arguments ##
    user_options_custom = [
        ("docker", None, "installing in Docker"),
    ]
    user_options = getattr(develop, "user_options", []) + user_options_custom

    def initialize_options(self):
        """Initialize extra argument options."""
        develop.initialize_options(self)

        ## Initialize options. ##
        self.docker = False

    def finalize_options(self):
        """Set extra install arguments."""
        develop.finalize_options(self)

    def run(self):
        """Overload the :py:meth:`setuptools.command.develop.develop.run` method."""

        ## Install panda_autograsp module ##
        subprocess.call([sys.executable, "-m", "pip", "install", os.getcwd()+"/panda_autograsp/"])

        ## Install submodules ##
        install_submods(sub_mods, "develop", install_dependencies=False)

        ## Run parent run method ##
        develop.run(self) ## Disabled because no top level python packages are present

        ## Print success message ##
        logger.info("All the python packages in this catkin_ws and their requirements have"
                    "been installed successfully.")

class InstallCmd(install, object):
    """Overload :py:class:`setuptools.command.install.install` class"""

    ## Add extra user arguments ##
    user_options_custom = [
        ("docker", None, "installing in Docker"),
    ]
    user_options = getattr(install, "user_options", []) + user_options_custom

    def initialize_options(self):
        """Initialize extra argument options."""
        install.initialize_options(self)

        ## Initialize options.
        self.docker = False

    def finalize_options(self):
        """Set extra argument options."""
        install.finalize_options(self)

    def run(self):
        """Overload the :py:meth:`setuptools.command.install.install.run` method."""

        ## Install panda_autograsp module #
        subprocess.call([sys.executable, "-m", "pip", "install", os.getcwd()+"/panda_autograsp/"])

        ## Install submodules #
        install_submods(sub_mods, "install", install_dependencies=False)

        ## Run parent run method ##
        install.run(self) # Disabled because no top level python packages are present

        ## Print success message ##
        logger.info("All the python packages in this catkin_ws and their requirements have"
                    "been installed successfully.")

#################################################
## Setup script #################################
#################################################

## Get current package version ##
__version__ = re.sub(r'[^\d.]', '', open(
    os.path.join(os.path.dirname(os.path.realpath(__file__)),
                 "panda_autograsp/version.py")).read())

## Add submodule requirements to the requirements list ##
requirements, sub_mods, sub_mods_egg_names = add_submods_requirements(requirements)
requirements = [j for j in requirements if not any([i.replace("_","-") in j for i in sub_mods])] # Remove submodule packages from requirements

## Run python setup ##
setup(
    name="panda_autograsp_ws",
    version=__version__,
    description=(
        "Project code for the panda_autograsp automatic grasping solution."),
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
        "Topic :: Scientific/Engineering"
    ],
    packages=find_packages(),
    include_package_data=True,
    install_requires=requirements,
    extras_require={
        "docs": ["sphinx", "sphinxcontrib-napoleon", "sphinx_rtd_theme", "sphinx-navtree", "sphinx-autobuild", "docutils", "doc8"],
        "dev": ["pytest", "bumpversion", "pylint"]
    },
    cmdclass={
        "install": InstallCmd,
        "develop": DevelopCmd
    }
)
