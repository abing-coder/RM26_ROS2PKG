# coding: utf-8

"""
"""

from can import CanError


class BmError(CanError):
    def __init__(self, bm_error_code, error_string, function):
        self.bm_error_code = bm_error_code
        text = "%s failed (%s)" % (function, error_string)
        super(BmError, self).__init__(text)


