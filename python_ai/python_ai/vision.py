"""Minimal vision helper for prototypes.

This provides a tiny `find_bright_spot` function used by a unit test. It is
pure-Python and fast to run in CI.
"""
from typing import List, Sequence


def find_bright_spot(pixels: Sequence[int]) -> int:
    """Return index of brightest pixel (max value). Empty -> -1."""
    if not pixels:
        return -1
    max_idx = 0
    max_val = pixels[0]
    for i, v in enumerate(pixels):
        if v > max_val:
            max_val = v
            max_idx = i
    return max_idx
