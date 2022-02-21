import pytest
import nelen_code

from nelen_code.utils.exceptions import raise_except

class dummy_axis:
    state=0
dummy_axis=dummy_axis()


def exception_raise():
    dummy_axis.state=0
    assert raise_except(dummy_axis, 1)
