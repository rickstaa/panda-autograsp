from panda_autograsp import Logger
import logging

## Set main logger ##
# jan = Logger.get_logger("tests")
# logging.getLogger()
# jan = logging.getLogger("test")
# jan.info("tests")
Logger.get_logger()
test = Logger.get_logger("test")
Logger.add_log_file("test.txt", test)
test.info("jan")
print("Jan")