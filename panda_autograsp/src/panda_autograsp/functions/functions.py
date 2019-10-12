#!/usr/bin/env python3
"""This modules contains some additional functions which are not part of a specific module.
"""

## System imports ##
from ..loggers import Logger
from autolab_core import YamlConfig
import os
import sys
import subprocess
import contextlib
import io
import logging
import re

## BerkeleyAutomation modules ##
logger = logging.getLogger()
logger.disabled = True  # Done to suppress perception warnings
logger.disabled = False

## import custom modules ##

## Create logger ##
func_log = Logger.get_logger(__name__)

## Read panda_autograsp configuration file ##
MAIN_CFG = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
	os.path.realpath(__file__)), "../../../cfg/main_config.yaml")))

## Create script contants ##
MODELS_PATH = os.path.abspath(os.path.join(
	os.path.dirname(os.path.realpath(__file__)), "../..", MAIN_CFG["defaults"]["models_dir"]))
DEFAULT_DOWNLOAD_SCRIPT_PATH = os.path.abspath(os.path.join(os.path.dirname(
	os.path.realpath(__file__)), "../../../gqcnn/scripts/downloads/models/download_models.sh"))
MODEL_RENAME_DICT = {"GQCNN-2.0": "GQ-Image-Wise",
                     "GQCNN-2.1": "GQ-Bin-Picking-Eps90",
                     "GQCNN-3.0": "GQ-Suction"}

#################################################
## Functions ####################################
#################################################


def download_model(model, model_output=MODELS_PATH, download_script_path=DEFAULT_DOWNLOAD_SCRIPT_PATH):
	"""This function downloads the Pretrained CNN models that are used in the :py:mod:`panda_autograsp` package, when they
	are not yet present on the system.

	Parameters
	----------
	model : str
		Name of the model you want to download.
	model_output : str, optional
		Path to the folder in which you want to place the downloaded models, by default MODELS_PATH
	download_script_path : str, optional
		Path to the download description script, by default GQCNN_DOWNLOAD_SCRIPT_PATH
	Returns
	-------
	int
		+------------+----------------------------+
		| Error code | Description                |
		+------------+----------------------------+
		| 0          | Model download successfull.|
		+------------+----------------------------+
		| 1          | Model name invalid.        |
		+------------+----------------------------+
		| 2          | Model download error.      |
		+------------+----------------------------+
		| 3          | Model unzip error.         |
		+------------+----------------------------+
		| 4          | Something else went wrong. |
		+------------+----------------------------+
	"""

	## Create model folder if it does not exists ##
	if not os.path.exists(model_output):
		os.makedirs(model_output)
		msg = "Creating model folder in panda_autograsp root directory."
		func_log.info(msg)

	## Generate model directory string ##
	model_dir = os.path.abspath(os.path.join(model_output, model))

	## Check if model is already present ##
	if not os.path.exists(model_dir):

		## Download start message ##
		func_log.info("Downloading the " + model + " model.")
		## Get solution group ##
		solution = [s for s in list(
			MAIN_CFG["grasp_detection_solutions"].keys()) if s in model.lower()]
		if (len(solution) > 1 or len(solution) == 0):  # Check if model name is vallid
			msg = "Model name is invalid. Please check the name and try again."
			func_log.warning(msg)
			return 1

		## Get model download url ##
		if solution[0] == "gqcnn":

			## Get right model download url out of the included download script ##
			with open(download_script_path) as file:
				for line in file:
					if "/"+model in line:
						model_download_url = [item for item in line.split(
							" ") if ("https://" in item)][0]

		## Check if model is available ##
		if not 'model_download_url' in locals():
			msg = "Model download url could not be found. Please check the model name and try again."
			func_log.warning(msg)
			return 1

		## Download model ##
		try:
			subprocess.check_call(["wget", "-O", os.path.abspath(
				os.path.join(model_output, model+'.zip')), model_download_url.rstrip()])
		except subprocess.CalledProcessError:
			msg = "Model download failed. Please check your internet connection and try again."
			func_log.warning(msg)
			return 2

		## Unzip model ##
		try:
			subprocess.check_call(["unzip", "-o", "-a", "-d", model_output,
                          os.path.abspath(os.path.join(model_output, model+'.zip'))])
			# remove zip file
			os.remove(os.path.abspath(os.path.join(
				model_output, model+'.zip')))
		except subprocess.CalledProcessError:
			msg = "Model unzip failed."
			func_log.warning(msg)
			return 3

		## Rename some model folders ##
		if model in MODEL_RENAME_DICT.keys():
			subprocess.check_call(["mv", os.path.abspath(os.path.join(
				model_output, MODEL_RENAME_DICT[model])), os.path.abspath(os.path.join(model_output, model))])

		## Return success boolean ##
		msg = model + " was downloaded successfully."
		func_log.info(msg)
		return 0
	else:
		msg = model + " was already present and thus not downloaded."
		func_log.info(msg)
		return 0


def list_files(path='.', exclude=[], recursive=True):
	"""Returns a list of files that are present in a folder.
	Parameters
	----------
	path : str, optional
		Parent folder of which you want to list the files, by default '.' meaning
		the current working directory.
	exclude : list, optional
		A list of files you want to exclude, by default []
	recursive : bool, optional
		Option specifying whether you also want to list files of subfolders, by default True
	level : int, optional
		If recursive is enabled this specifies up till how many levels deep you want to list
		the files, by default 0 (Defined as all levels).
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
			rel_file = os.path.join(rel_dir, file_name)

			## Add files to file list if they are not in exclude list ##
			if file_name not in exclude:
				file_list.append(rel_file)

		## Break out of loop if recursive is disabled
		if not recursive:
			break
	return file_list


def yes_or_no(question, add_options=True):
	"""Simple yes or no prompt.

	Parameters
	----------
	question : str
		Question of the yes or no prompt.
	add_options : bool
		Include option statement "(Y/n)" after the question.

	Returns
	-------
	bool
		Boolean specifying if the user gave the right response.
	"""

	## Check add_options argment ##
	if add_options:
		option_str = " (Y/n)>> "
	else:
		option_str = ""

	## Create prompt ##
	answer = raw_input(question + option_str).lower().strip()
	answer = [answer] if answer == "" else answer  # Make sure enter i

	## Keep repeating prompt until right answer is given ##
	while not(answer == "y" or answer == "yes" or
           answer == "n" or answer == "no" or answer[0] == ""):
		print("")
		print("Input yes or no")
		answer = raw_input(question + option_str).lower().strip()
		answer = [answer] if answer == "" else answer  # Make sure enter i

	## Check answer ##
	if answer[0] == "y" or answer[0] == "":
		return True
	else:
		return False
