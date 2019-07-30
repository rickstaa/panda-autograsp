"""This is a test script to get the download url out of the model_download script.
"""

## Imports ##
import subprocess
import os
import sys

## Settings ##
MODEL = "GQCNN-2.0"
MODELS_PATH = os.path.abspath("./gqcnn/models")
MODEL_PATH = os.path.abspath(os.path.join(MODELS_PATH, MODEL))
DOWNLOAD_SCRIPTH_PATH = "./gqcnn/scripts/downloads/models/download_models.sh"
MODEL_RENAME_DICT = {"GQCNN-2.0": ["GQ-Image-Wise", "GQCNN-2.0"],
                     "GQCNN-2.1": ["GQ-Bin-Picking-Eps90", "GQCNN-2.1"],
                     "GQCNN-3.0": ["GQ-Suction", "GQCNN-3.0"]}

## Load file ##
data = []
with open(DOWNLOAD_SCRIPTH_PATH) as file:

    ## Get right model download url ##
    for line in file:
        if "/"+MODEL in line:
            model_download_url = [item for item in line.split(
                " ") if ("https://" in item)][0]

## Create model folder if it does not exists ##
if not os.path.exists(os.path.abspath(os.path.join(MODELS_PATH, MODEL))):
    os.makedirs(os.path.abspath(os.path.join(MODELS_PATH, MODEL)))

## Download model ##
try:
    proc = subprocess.check_call(["wget", "-O", os.path.abspath(
        os.path.join(MODELS_PATH, MODEL+'.zip')), model_download_url.rstrip()])
except subprocess.CalledProcessError:
    print("Models could not be downloaded. Please check your internet connection and try again.")
    print("Shutting down panda_autograsp algorithm.")
    sys.exit(0)

## Unzip model ##
try:
    proc = subprocess.check_call(["unzip", "-o", "-a", "-d", os.path.abspath(
        MODEL_PATH), os.path.abspath(os.path.join(MODEL_PATH, MODEL+'.zip'))])
except subprocess.CalledProcessError:
    print("Model could not be unzipped. Please check the model download folder at (%s).", MODEL_PATH)
    print("Shutting down panda_autograsp algorithm.")
    sys.exit(0)

## Rename some model folders ##
if MODEL in MODEL_RENAME_DICT.keys():
    proc = subprocess.check_call(["mv", os.path.abspath(os.path.join(
        MODELS_PATH, MODEL_RENAME_DICT[MODEL][0])), os.path.abspath(os.path.join(MODELS_PATH, MODEL_RENAME_DICT[MODEL][1]))])

## Get the right data download url out of the download file ##
print("I'm here")
