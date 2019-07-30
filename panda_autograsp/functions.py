"""This modules contains some additional functions which are not part of a specific module.
"""

## System imports ##
import os
import sys
import subprocess
import contextlib
import io

## BerkeleyAutomation modules ##
from autolab_core import YamlConfig

## import custom modules ##
from panda_autograsp import Logger

## Create logger ##
func_log = Logger.get_logger(__name__)

## Read panda_autograsp configuration file ##
main_cfg = YamlConfig(os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../cfg/main_config.yaml")))

## Create script contants ##
MODELS_PATH = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", main_cfg["defaults"]["models_dir"]))
GQCNN_DOWNLOAD_SCRIPT_PATH = os.path.abspath(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), "../gqcnn/scripts/downloads/models/download_models.sh"))
GQCNN_MODEL_RENAME_DICT = {"GQCNN-2.0": "GQ-Image-Wise",
                           "GQCNN-2.1": "GQ-Bin-Picking-Eps90", "GQCNN-3.0": "GQ-Suction"}

#################################################
## Functions ####################################
#################################################
def download_model(model):
    """This function downloads the Pretrained CNN models that are used in the :py:mod:`panda_autograsp` package, when they
    are not yet present on the system.

    Parameters
    ----------
    model : str
        Name of the model you want to download.

    Returns
    -------
    int
        +------------+----------------------------+
        | Error code | Description                |
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
    if not os.path.exists(MODELS_PATH):
        os.makedirs(MODELS_PATH)
        msg = "Creating model folder in panda_autograsp root directory."
        func_log.info(msg)

    ## Generate model directory string ##
    model_dir = os.path.abspath(os.path.join(MODELS_PATH, model))

    ## Check if model is already present ##
    if not os.path.exists(model_dir):

        ## Download start message ##
        func_log.info("Downloading the " + model + " model.")
        ## Get solution group ##
        solution = [s for s in list(main_cfg["grasp_detection_solutions"].keys()) if s in model.lower()]
        if (len(solution) > 1 or len(solution) == 0):  # Check if model name is vallid
            msg = "Model name is invalid. Please check the name and try again."
            func_log.warning(msg)
            return 1

        ## Get model download url ##
        if solution[0] == "gqcnn":

            ## Get right model download url out of the included download script ##
            with open(GQCNN_DOWNLOAD_SCRIPT_PATH) as file:
                for line in file:
                    if "/"+model in line:
                        model_download_url = [item for item in line.split(
                            " ") if ("https://" in item)][0]

        ## Download model ##
        try:
            subprocess.check_call(["wget", "-O", os.path.abspath(
                os.path.join(MODELS_PATH, model+'.zip')), model_download_url.rstrip()])
        except subprocess.CalledProcessError:
            msg = "Model download failed. Please check your internet connection and try again."
            func_log.warning(msg)
            return 2

        ## Unzip model ##
        try:
            subprocess.check_call(["unzip", "-o", "-a", "-d", MODELS_PATH,
                                   os.path.abspath(os.path.join(MODELS_PATH, model+'.zip'))])
            # remove zip file
            os.remove(os.path.abspath(os.path.join(
                MODELS_PATH, model+'.zip')))
        except subprocess.CalledProcessError:
            msg = "Model unzip failed."
            func_log.warning(msg)
            return 3

        ## Rename some model folders ##
        if model in GQCNN_MODEL_RENAME_DICT.keys():
            subprocess.check_call(["mv", os.path.abspath(os.path.join(
                MODELS_PATH, GQCNN_MODEL_RENAME_DICT[model])), os.path.abspath(os.path.join(MODELS_PATH, model))])

        ## Return success boolean ##
        msg = model + " was downloaded successfully."
        func_log.info(msg)
        return
    else:
        msg = model + " was already present and thus not downloaded."
        func_log.info(msg)
        return

@contextlib.contextmanager
def nostdout():
    """Create contex to silence the stdout"""
    save_stdout = sys.stdout
    sys.stdout = io.BytesIO()
    yield
    sys.stdout = save_stdout
