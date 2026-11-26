"""Your ArUco detector for the Lab 7 camera."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


def detect_markers(
    image: NDArray[np.uint8],
    dictionary_name: str,
) -> tuple[list[int], list[NDArray[np.float32]]]:
    """Find the ArUco markers in one camera frame.

    Inputs:
        ``image`` is a BGR array of shape (height, width, 3).
        ``dictionary_name`` names an OpenCV dictionary such as DICT_4X4_50.
        The caller validates the image and dictionary before calling you.

    Return:
        Two lists containing Python integer IDs and corresponding OpenCV
        corner arrays of shape (1, 4, 2). Sort by ID and keep each marker's
        corners with its ID. Return ([], []) when no marker is visible.
        Return any ID the dictionary recognises. The caller looks up labels.
    """
    # load the configured opencv aruco dictionary

    # detect markers in this image using the aruco api on the lab machine

    # keep the ids and corner arrays paired when sorting
    raise NotImplementedError("Implement detect_markers in exercises/aruco_reader.py")
