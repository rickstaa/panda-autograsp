"""Perform some tests for the gqcnn test_module
"""

import gqcnn

def test_gqcnn():
    """Test whether gqcnn was imported correctly
    """

    ## Check if gqcnn is imported correctly ##
    try:
        gqcnn
    except NameError:
        raise AssertionError