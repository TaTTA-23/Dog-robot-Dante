from python_ai.vision import find_bright_spot


def test_find_bright_spot_empty():
    assert find_bright_spot([]) == -1


def test_find_bright_spot_basic():
    data = [10, 250, 100, 250]
    # first max is index 1
    assert find_bright_spot(data) == 1
import numpy as np
from ai_dante.vision import find_brightest_pixel


def test_find_brightest_pixel_gray():
    img = np.array([[0, 1], [2, 3]])
    assert find_brightest_pixel(img) == (1, 1)


def test_find_brightest_pixel_color():
    # brightest at (0,2)
    img = np.zeros((2,3,3), dtype=np.uint8)
    img[0,2] = [10, 20, 30]
    assert find_brightest_pixel(img) == (0, 2)
