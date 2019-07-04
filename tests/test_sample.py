"""
Content of test_sample.py
"""
def func(x):
    """
    Function to be tested
    """
    return x + 1

def test_answer():
    """
    Test function
    """
    assert func(3) == 4
