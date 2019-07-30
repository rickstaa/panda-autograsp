"""Perform some tests for the gqcnn test_module
"""

import panda_autograsp

def test_panda_autograsp():
    """Test whether gqcnn was imported correctly
    """

    ## Check if gqcnn is imported correctly ##
    try:
        panda_autograsp
    except NameError:
        raise AssertionError