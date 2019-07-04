### Import custom project modules ###
from phalconsimulation import phalcon_camera
from phalconsimulation import phalcon_worker
from ptcsimulation import LoggerWriter          # Class created to log to the console and file simultaneously

from configobj import ConfigObj
import numpy as np
import multiprocessing as mp
import logging, time, os, sys
import serial
import time

### Import modules needed for multiprocessing logging ##
import traceback
import threading, logging, sys
from ptcsimulation import SubProcessLogHandler, LogQueueReader

###############################################################################
### Script settings                                                        ####
###############################################################################
queue = mp.Queue()
### General script settings ###
config_file_path = r"C:\\Repositories\\SOL_PTC_Vortex_Simulator\\etc\\PTC_config.ini"

### Logging settings ###
logPath = r"C:\Repositories\SOL_PTC_Vortex_Simulator\logs"
log_file_name = r"phalcon_simulation_vortex_logger"
DEFAULT_LEVEL = logging.DEBUG
log_formatter = logging.Formatter("%(asctime)s [%(processName)s] [%(levelname)s]:  - %(name)s - %(process)s - %(message)s")
log_file_path = "{0}/{1}.log".format(logPath, log_file_name)

###########################################################################
### Set multiprocessing interperter settings                            ###
###########################################################################
# The Python script needs to be in its own file (not just in the Code     #
# parameter of the Script extension in the Vortex Studio Editor)          #
# as the forked process needs to refer to it.                             #
###########################################################################
sys.argv = [ r"C:\Repositories\SOL_PTC_Vortex_Simulator\scripts\TestMultiProcessingPhalcon.py"]

# mp.set_executable(os.path.join(sys.exec_prefix, 'pythonw.exe'))  # Invisible python interperter
mp.set_executable(os.path.join(sys.exec_prefix, 'python.exe'))     # Visible python interperter
calculationStartTime = -1                                     # Make sure start time is correct

###########################################################################
### Read Configuration File                                             ###
###########################################################################
try:
    Config = ConfigObj(config_file_path)
    # root_logger.info("Opening config file: successful")
    print "dit is raar"
    print Config
    print "Opening config file: successful"
except:
    # root_logger.warning("Error reading Config file PTCMainSCript")
    print "Error reading Config file PTCMainSCript"

###########################################################################
### Get phalcon parameters out of Config file                           ###
###########################################################################
UseCalibrationIRBeacon = int(Config["Phalcon"]["use_calibration_ir_beacon"])

###########################################################################
### Initiate Phalcon camera simulation                                  ###
###########################################################################

###########################################################################
### Initiate shared memory marker data array                            ###
###########################################################################

### Create array shape specifier ###
X_shape = (15, 6)

### Create multiprocessing array ###
X = mp.Array('d', X_shape[0] * X_shape[1])

### Wrap X as an numpy array so we can easily manipulates its data. ###
X_np = np.frombuffer(X.get_obj()).reshape(X_shape)

### Put meaningfull data on shared memory address (the phalcon algorithm can deal with a fully empty matrix) ##
camera_data_start = np.random.randn(*X_shape)
np.copyto(X_np, camera_data_start)

###########################################################################
### Initiate multiprocessing process thread                             ###
###########################################################################
phalcon_worker = phalcon_worker.PhalconWorker(X, X_shape, Config, queue)

### Start thread ###
if __name__ == '__main__':
    print "phalcon_worker.start()"
    phalcon_worker.start()


time.sleep(15)   # Delays for 5 seconds. You can also use a float value.
# phalcon_worker.join()
# print "spawning process exits"


# ### DEBUG: ###
# except Exception as ex:
#     pass