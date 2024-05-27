"""
Sample test class, delete when tests are written
"""

import pytest


class TestSample:
    """
    This class runs a basic test
    """

    def test_add(self):
        """
        Test addition
        """
        input_1 = 1
        input_2 = 2
        expected = 3

        actual = input_1 + input_2

        assert actual == expected
