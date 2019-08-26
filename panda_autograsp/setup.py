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
from distutils.sysconfig import get_python_lib

## Cat required paths ##
relative_site_packages = get_python_lib().split(sys.prefix+os.sep)[1]
date_files_relative_path = os.path.join(relative_site_packages, "panda_autograsp")

## Package requirements ##
setup_requirements = []
requirements = ["pylibfreenect2", "tensorflow-estimator >= 1.13.0, <1.14.0rc0"]

## General setup.py parameters ##
TF_MAX_VERSION = "1.13.1"

## Set up logger. ##
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#################################################
## Setup functions ##############################
#################################################
def get_tf_dep():
	"""Check whether or not the Nvidia driver and GPUs are available and add the
	corresponding Tensorflow dependency."""

	## Find the right tensorflow version to install ##
	tf_dep = "tensorflow<={}".format(TF_MAX_VERSION)
	try:
		gpus = subprocess.check_output(
			["nvidia-smi", "--query-gpu=gpu_name",
			 "--format=csv"]).decode().strip().split("\n")[1:]
		if len(gpus) > 0:
			tf_dep = "tensorflow-gpu<={}".format(TF_MAX_VERSION)
		else:
			no_device_msg = ("Found Nvidia device driver but no"
							 " devices...installing Tensorflow for CPU.")
			logger.warning(no_device_msg)
	except OSError:
		no_driver_msg = ("Could not find Nvidia device driver...installing"
						 " Tensorflow for CPU.")
		logger.warning(no_driver_msg)
	return tf_dep

def list_files(path='.', exclude=[], recursive=True, prepent_parent=False):
	"""Returns a list of files that are present in a folder.

	Parameters
	----------
	path : str, optional
		Parent folder of which you want to list the files, by default '.'
	exclude : list, optional
		A list of files you want to exclude, by default []
	recursive : bool, optional
		Option specifying whether you also want to list files of subfolders, by default True
	level : int, optional
		If recursive is enabled this specifies up till how many levels deep you want to list
		the files, by default 0 (Defined as all levels).
	perpent_parent: bool, optional
		Options specifying whether you want to prepent the parent dir to the output paths.

	Returns
	-------
	List
		A list containing the relative paths of all the files in the parent folder.
	"""

	## Get a list of files that are contained in the given path
	file_list = list()
	for dir_, _, files in os.walk(path):
		for file_name in files:
			rel_dir = os.path.relpath(dir_, path)

			## Prepent parent folder if specified ##
			if not prepent_parent:
				rel_file = os.path.join(rel_dir, file_name)
			else:
				rel_file = os.path.join(dir_, file_name).replace("./","")

			## Add files to file list if they are not in exclude list ##
			if file_name not in exclude:
				file_list.append(rel_file)

		## Break out of loop if recursive is disabled
		if not recursive:
			break
	return file_list

#################################################
## Setup classes ################################
#################################################
class DevelopCmd(develop):
	"""Overload :py:class:`setuptools.command.develop.develop` class."""

	user_options_custom = [
		("docker", None, "installing in Docker"),
		("sing", None, "installing in Singularity")
	]
	user_options = getattr(develop, "user_options", []) + user_options_custom

	def initialize_options(self):
		develop.initialize_options(self)

		# Initialize options.
		self.docker = False
		self.sing = False

	def finalize_options(self):
		develop.finalize_options(self)

	def run(self):
		"""Overload the :py:meth:`setuptools.command.develop.develop.run` method."""

		# Install Tensorflow dependency.
		if not self.docker and not self.sing:
			tf_dep = get_tf_dep()
			subprocess.Popen([sys.executable, "-m", "pip", "install",
							  tf_dep]).wait()
		else:
			# If we're using a Docker or singularity container, the right
			# tensoflow version is specified in the recipe file. This is
			# done since there is no way to check for CUDA/AMD GPU's at
			# container build time.
			skip_tf_msg = ("Omitting Tensorflow dependency because of Docker"
						   " installation.")
			logger.warning(skip_tf_msg)

		# Run installation.
		develop.run(self)

class InstallCmd(install, object):
	"""Overload :py:class:`setuptools.command.install.install` class"""

	## Add extra user arguments ##
	user_options_custom = [
		("docker", None, "installing in Docker"),
		("sing", None, "Installing in Singularity")
	]
	user_options = getattr(install, "user_options", []) + user_options_custom

	def initialize_options(self):
		"""Initialize extra argument options."""
		install.initialize_options(self)

		# Initialize options.
		self.docker = False
		self.sing = False

	def finalize_options(self):
		"""Set extra argument options."""
		install.finalize_options(self)

	def run(self):
		"""Overload the :py:meth:`setuptools.command.install.install.run` method."""

		# Install Tensorflow dependency.
		if any(['tensorflow' in item for item in self.distribution.install_requires]):
			if not self.docker and not self.sing:
				tf_dep = get_tf_dep()
				subprocess.Popen([sys.executable, "-m", "pip", "install",
								  tf_dep]).wait()
			else:
				# If we're using a Docker or singularity container, the right
				# tensoflow version is specified in the recipe file. This is
				# done since there is no way to check for CUDA/AMD GPU's at
				# container build time.
				skip_tf_msg = ("Omitting Tensorflow dependency because of Docker"
							   " installation.")
				logger.warning(skip_tf_msg)

		## Run parent run method ##
		install.run(self)

#################################################
## Setup script #################################
#################################################

## Get current package version ##
__version__ = re.sub(r'[^\d.]', '', open(
	os.path.join(os.path.dirname(os.path.realpath(__file__)),
				 "version.py")).read())

## Parse readme.md ##
with open("README.md") as f:
	readme = f.read()

## Run python setup ##
setup(
	name="panda_autograsp",
	version=__version__,
	description=(
		"Project code for the panda_autograsp automatic grasping solution."),
	long_description=readme,
	long_description_content_type='text/markdown',
	author="Rick Staa",
	author_email="rick.staa@outlook.com",
	license="Rick Staa Copyright",
	url="https://github.com/rickstaa/panyda_autograsp",
	keywords="robotics grasping vision deep learning franka gqcnn gpd",
	classifiers=[
		"Programming Language :: Python :: 3.5",
		"Programming Language :: Python :: 3.6",
		"Programming Language :: Python :: 3.7",
		"Natural Language :: English",
		"Topic :: Scientific/Engineering"
	],
	package_dir={'': 'src'},
	packages=find_packages(where='src'),
	install_requires=requirements,
	extras_require={
		"docs": ["sphinx", "sphinxcontrib-napoleon", "sphinx_rtd_theme", "sphinx-navtree", "sphinx-autobuild", "docutils", "doc8"],
		"dev": ["pytest", "bumpversion", "pylint"]
	},
	include_package_data=True,
	data_files=[(date_files_relative_path, ['README.md']),
	  			  (os.path.join(date_files_relative_path,'cfg'), list_files('cfg', recursive=False, prepent_parent=True)),
				  (os.path.join(date_files_relative_path,'cfg', '_cfg'), list_files('cfg/_cfg', recursive=False, prepent_parent=True)),
				  (os.path.join(date_files_relative_path,'data', 'calib'), list_files('data/calib', recursive=False, prepent_parent=True))
				],
	cmdclass={
		"install": InstallCmd,
		"develop": DevelopCmd
	}
)
