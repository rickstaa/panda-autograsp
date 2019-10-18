#!/usr/bin/env python3
"""Utility class for logging. This class is a wrapper around the original
logging module :py:mod:`logging` and can be used to apply the panda_autograsp
formatters, filters and handlers to the logging object.
"""

# Main python packages
import logging
import sys
import os
import colorlog

#################################################
# Script parameters #############################
#################################################
ROOT_LOG_LEVEL = logging.INFO
ROOT_LOG_STREAM = sys.stdout


#################################################
# Configure root logger function ################
#################################################
def clear_root():
    """Function used to reset the root logger."""
    root_logger = logging.getLogger()

    # clear any existing handles to streams because we don't want duplicate logs
    # NOTE: we assume that any stream handles we find are to ROOT_LOG_STREAM,
    # which is usually the case(because it is stdout). This is fine because we
    # will be re-creating that handle. Otherwise we might be deleting a handle
    # that won't be re-created, which could result in dropped logs.
    for hdlr in root_logger.handlers:
        if isinstance(hdlr, logging.StreamHandler):
            root_logger.removeHandler(hdlr)

    # create nullhandler to suppress no handler warning
    root_logger.addHandler(logging.NullHandler())

    # Set root configured to true
    Logger.ROOT_CONFIGURED = False


#################################################
# Configure root logger function ################
#################################################
def configure_root(log_level=ROOT_LOG_LEVEL):
    """Function used to configure the root logger.

    Parameters
    ----------
    log_level : :py:obj:`python2.int`, optional
        [description], by default ROOT_LOG_LEVEL

    Returns
    -------
    :py:obj:`python2.Logger`
        Root logger.
    """
    root_logger = logging.getLogger()

    # clear any existing handles to streams because we don't want duplicate logs
    # NOTE: we assume that any stream handles we find are to ROOT_LOG_STREAM,
    # which is usually the case(because it is stdout). This is fine because we
    # will be re-creating that handle. Otherwise we might be deleting a handle
    # that won't be re-created, which could result in dropped logs.
    for hdlr in root_logger.handlers:
        if isinstance(hdlr, logging.StreamHandler):
            root_logger.removeHandler(hdlr)

    # configure the root logger
    root_logger.setLevel(log_level)
    hdlr = logging.StreamHandler(ROOT_LOG_STREAM)
    formatter = colorlog.ColoredFormatter(
        (
            "%(blue)s%(name)-10s %(log_color)s%(levelname)-8s%(reset)s "
            "%(white)s%(message)s"
        ),
        reset=True,
        log_colors={
            "DEBUG": "cyan",
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
    )
    hdlr.setFormatter(formatter)
    root_logger.addHandler(hdlr)

    # Set root configured to true
    Logger.ROOT_CONFIGURED = True
    return root_logger


#################################################
# Root Logger file handler add ##################
#################################################
def add_root_log_file(log_file, mode="a", encoding=None, delay=False):
    """
    Add a log file to the root logger.

    Parameters
    ----------
    log_file :py:obj:`python2.str`
        The path to the log file.
    mode :py:obj:`python2.str`
        Log file writing mode, by default 'a'.
    encoding: :py:obj:`python2.str`
        File encoding used, by default None.
    delay: :py:obj:`python2.str`
        If delay is true, then file opening is deferred until the first call
        to emit(), by default False.
    """
    root_logger = logging.getLogger()

    # Create model folder if it does not exists
    log_folder = os.path.abspath(os.path.join(log_file, os.pardir))
    if not os.path.exists(log_folder):
        try:
            os.makedirs(log_folder)
        except OSError:
            root_logger.info(
                "Log file could not be created logger not logging to file {}".format(
                    log_file
                )
            )
            return

    # Add a file handle to the root logger
    hdlr = logging.FileHandler(log_file, mode, encoding, delay)
    formatter = logging.Formatter(
        "%(asctime)s %(name)-10s %(levelname)-8s %(message)s", datefmt="%m-%d %H:%M:%S"
    )
    hdlr.setFormatter(formatter)
    root_logger.info("Root logger now logging to {}".format(log_file))
    root_logger.addHandler(hdlr)


#################################################
# Logger class ##################################
#################################################
class Logger(object):
    ROOT_CONFIGURED = False

    @staticmethod
    def clear_root():
        """Reset root logger."""
        clear_root()

    @staticmethod
    def reconfigure_root():
        """Reconfigure the root logger."""
        configure_root()

    @staticmethod
    def get_logger(
        name=None,
        log_level=logging.INFO,
        log_file=None,
        silence=False,
        mode="a",
        encoding=None,
        delay=False,
    ):
        """
        Build a logger. All logs will be propagated up to the root logger if not
        silenced. If log_file is provided, logs will be written out to that file.
        If no logger name is given, log_file will be handed the root logger,
        otherwise it will only be used by this particular logger.

        Parameters
        ----------
        name :py:obj:`python2.str`
            The name of the logger to be built, by default "" thus formatting the
            root logger.
        log_level : `python2.int`
            The log level. See the python logging module documentation for possible
            enum values.
        log_file :py:obj:`python2.str`
            The path to the log file to log to.
        silence :py:obj:`python2.bool`
            Whether or not to silence this logger. If it is silenced, the only way
            to get output from this logger is through a non-global log file.
        mode :py:obj:`python2.str`
            Log file writing mode, by default 'a'.
        encoding: :py:obj:`python2.str`
            File encoding used, by default None.
        delay: :py:obj:`python2.str`
            If delay is true, then file opening is deferred until the first call
            to emit(), by default False.

        Returns
        -------
        :py:obj:`python2.Logger`
            A custom logger.
        """

        # Create a new logger object with the panda_autograsp formatting
        if not name:  # Format the root logger

            # some checks for silencing/no-op logging
            if silence:
                raise ValueError(
                    "You can't silence a logger and log to a global log file!"
                )

            # Setup root_logger
            if not Logger.ROOT_CONFIGURED:
                root_logger = configure_root(log_level)
            Logger.ROOT_CONFIGURED = True

            # configure the log file stream
            if log_file is not None:
                add_root_log_file(log_file, mode, encoding, delay)

            # Return root logger
            return root_logger
        else:  # Create new logger object
            no_op = False

            # some checks for silencing/no-op logging
            if silence and log_file is None:
                logging.warning("You are creating a no-op logger!")
                no_op = True

            # build a logger
            logger = logging.getLogger(name)
            logger.setLevel(log_level)

            # silence the logger by preventing it from propagating upwards to the root
            logger.propagate = not silence

            # configure the log file stream
            if log_file is not None:

                # Add logger file handler #
                hdlr = logging.FileHandler(log_file, mode, encoding, delay)
                formatter = logging.Formatter(
                    "%(asctime)s %(name)-10s %(levelname)-8s %(message)s",
                    datefmt="%m-%d %H:%M:%S",
                )
                hdlr.setFormatter(formatter)
                logger.addHandler(hdlr)

            # add a no-op handler to suppress warnings about there being no handlers
            if no_op:
                logger.addHandler(logging.NullHandler())
            return logger

    @staticmethod
    def add_log_file(log_file=None, logger=None, mode="a", encoding=None, delay=False):
        """
        Add a log file to this logger. If no logger is given, log_file will be
        handed the root logger, otherwise it will only be used by this particular
        logger.

        Parameters
        ----------
        log_file :py:obj:`python2.str`
            The path to the log file to log to.
        logger :py:obj:`logging.Logger`
            The logger.
        mode :py:obj:`python2.str`
            Log file writing mode, by default 'a'.
        encoding: :py:obj:`python2.str`
            File encoding used, by default None.
        delay: :py:obj:`python2.str`
            If delay is true, then file opening is deferred until the first call
            to emit(), by default False.
        """

        # Add logfile to logger
        if logger:  # Add to root logger
            add_root_log_file(log_file, mode, encoding, delay)
        else:  # Add to specified logger
            hdlr = logging.FileHandler(log_file, mode, encoding, delay)
            formatter = logging.Formatter(
                "%(asctime)s %(name)-10s %(levelname)-8s %(message)s",
                datefmt="%m-%d %H:%M:%S",
            )
            hdlr.setFormatter(formatter)
            logger.addHandler(hdlr)
