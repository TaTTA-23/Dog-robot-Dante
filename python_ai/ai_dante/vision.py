"""Simple vision helpers used for prototypes."""

import numpy as np


def find_brightest_pixel(image: np.ndarray):
    """Return (row, col) of the brightest pixel in a grayscale or color image.

    Raises ValueError if image is empty or not a numpy array.
    """
    if not isinstance(image, np.ndarray):
        raise ValueError('image must be a numpy.ndarray')
    if image.size == 0:
        raise ValueError('image is empty')

    # If color, convert to grayscale by summing channels
    if image.ndim == 3:
        gray = image.sum(axis=2)
    else:
        gray = image

    idx = np.unravel_index(np.argmax(gray), gray.shape)
    return idx
