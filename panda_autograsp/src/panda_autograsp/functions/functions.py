#!/usr/bin/env python3
"""This modules contains some additional functions which are not part
of a specific module.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Main python packages
import os
import subprocess
import cv2

from autolab_core import YamlConfig

# Panda_autograsp modules, msgs and srvs
from ..loggers import Logger

# Create logger
func_log = Logger.get_logger(__name__)

# Read panda_autograsp configuration file
MAIN_CFG = YamlConfig(
    os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "../../../cfg/main_config.yaml"
        )
    )
)

# Create script contants
DEFAULT_MODELS_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../..",
        MAIN_CFG["main"]["models_dir"],
    )
)
DEFAULT_DOWNLOAD_SCRIPT_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../../../gqcnn/scripts/downloads/models/download_models.sh",
    )
)
MODEL_RENAME_DICT = {
    "GQCNN-2.0": "GQ-Image-Wise",
    "GQCNN-2.1": "GQ-Bin-Picking-Eps90",
    "GQCNN-3.0": "GQ-Suction",
}


#################################################
# Functions #####################################
#################################################
def download_model(
    model,
    model_output=DEFAULT_MODELS_PATH,
    download_script_path=DEFAULT_DOWNLOAD_SCRIPT_PATH,
):
    """This function downloads the Pretrained CNN models that are used in the
    ``panda_autograsp`` package, when they are not yet present on the system.

    Parameters
    ----------
    model : :py:obj:`str`
        Name of the model you want to download.
    model_output : :py:obj:`str`, optional
        Path to the folder in which you want to place the downloaded models, by
        default DEFAULT_MODELS_PATH
    download_script_path : :py:obj:`str`, optional
        Path to the download description script, by default GQCNN_DOWNLOAD_SCRIPT_PATH

    Returns
    -------
    :py:obj:`int`

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

    # Create model folder if it does not exists
    if not os.path.exists(model_output):
        os.makedirs(model_output)
        msg = "Creating model folder in panda_autograsp root directory."
        func_log.info(msg)

    # Generate model directory string
    model_dir = os.path.abspath(os.path.join(model_output, model))

    # Check if model is already present
    if not os.path.exists(model_dir):

        # Download start message
        func_log.info("Downloading the " + model + " model.")

        # Get solution group
        solution = [
            s for s in list(MAIN_CFG["grasp_detection"].keys()) if s in model.lower()
        ]
        if len(solution) > 1 or len(solution) == 0:  # Check if model name is vallid
            msg = "Model name is invalid. Please check the name and try again."
            func_log.warning(msg)
            return 1

        # Get model download url
        if solution[0] == "gqcnn":

            # Get right model download url out of the included download script
            with open(download_script_path) as file:
                for line in file:
                    if "/" + model in line:
                        model_download_url = [
                            item for item in line.split(" ") if ("https://" in item)
                        ][0]

        # Check if model is available
        if "model_download_url" not in locals():
            msg = (
                "Model download url could not be found. Please check th "
                "emodel name and try again."
            )
            func_log.warning(msg)
            return 1

        # Download model
        try:
            subprocess.check_call(
                [
                    "wget",
                    "-O",
                    os.path.abspath(os.path.join(model_output, model + ".zip")),
                    model_download_url.rstrip(),
                ]
            )
        except subprocess.CalledProcessError:
            msg = (
                "Model download failed. Please check your internet connection "
                "and try again."
            )
            func_log.warning(msg)
            return 2

        # Unzip model
        try:
            subprocess.check_call(
                [
                    "unzip",
                    "-o",
                    "-a",
                    "-d",
                    model_output,
                    os.path.abspath(os.path.join(model_output, model + ".zip")),
                ]
            )
            # remove zip file
            os.remove(os.path.abspath(os.path.join(model_output, model + ".zip")))
        except subprocess.CalledProcessError:
            msg = "Model unzip failed."
            func_log.warning(msg)
            return 3

        # Rename some model folders
        if model in MODEL_RENAME_DICT.keys():
            subprocess.check_call(
                [
                    "mv",
                    os.path.abspath(
                        os.path.join(model_output, MODEL_RENAME_DICT[model])
                    ),
                    os.path.abspath(os.path.join(model_output, model)),
                ]
            )

        # Return success boolean
        msg = model + " was downloaded successfully."
        func_log.info(msg)
        return 0
    else:
        msg = model + " was already present and thus not downloaded."
        func_log.info(msg)
        return 0


def list_files(path=".", exclude=[], recursive=True, prepent_parent=False):
    """Returns a list of files that are present in a folder.

    Parameters
    ----------
    path : :py:obj:`str`, optional
        Parent folder of which you want to list the files, by default ``.``
    exclude : :py:obj:`list`, optional
        A list of files you want to exclude, by default `[]`
    recursive : :py:obj:`bool`, optional
        Option specifying whether you also want to list files of subfolders,
        by default True
    level : :py:obj:`int`, optional
        If recursive is enabled this specifies up till how many levels deep you want
        to list the files, by default 0 (Defined as all levels).
    perpent_parent: :py:obj:`bool`, optional
        Options specifying whether you want to prepent the parent dir to the output
        paths.

    Returns
    -------
    :py:obj:`List`
        A list containing the relative paths of all the files in the parent folder.
    """

    # Get a list of files that are contained in the given path
    file_list = list()
    for dir_, _, files in os.walk(path):
        for file_name in files:
            rel_dir = os.path.relpath(dir_, path)

            # Prepent parent folder if specified
            if not prepent_parent:
                rel_file = os.path.join(rel_dir, file_name)
            else:
                rel_file = os.path.join(dir_, file_name).replace("./", "")

            # Add files to file list if they are not in exclude list
            if file_name not in exclude:
                file_list.append(rel_file)

        # Break out of loop if recursive is disabled
        if not recursive:
            break
    return file_list


def yes_or_no(question, add_options=True):
    """Simple yes or no prompt.

    Parameters
    ----------
    question : :py:obj:`str`
        Question of the yes or no prompt.
    add_options : :py:obj:`bool`
        Include option statement (Y/n) after the question.

    Returns
    -------
    :py:obj:`bool`
        Boolean specifying if the user gave the right response.
    """

    # Check add_options argment
    if add_options:
        option_str = " (Y/n)>> "
    else:
        option_str = ""

    # Create prompt
    answer = input(question + option_str).lower().strip()
    answer = [answer] if answer == "" else answer  # Make sure enter i

    # Keep repeating prompt until right answer is given
    while not (
        answer == "y"
        or answer == "yes"
        or answer == "n"
        or answer == "no"
        or answer[0] == ""
    ):
        print("")
        print("Input yes or no")
        answer = input(question + option_str).lower().strip()
        answer = [answer] if answer == "" else answer  # Make sure enter i

    # Check answer
    if answer[0] == "y" or answer[0] == "":
        return True
    else:
        return False


def draw_axis(img, corners, imgpts):
    """Takes the corners in the chessboard (obtained using cv2.findChessboardCorners())
    and axis points to draw a 3D axis.

    Parameters
    ----------
    img : :py:obj:`numpy.ndarray`
        Image
    corners : :py:obj:`pyton2.tuple`
        Chess board corner points.
    imgpts : :py:obj:`nmpy.ndarray`
        Axis points.

    Returns
    -------
    :py:obj:`numpy.ndarray`
        Image on which the chessboard corners have been drawn.
    """
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img
