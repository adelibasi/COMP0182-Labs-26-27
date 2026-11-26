"""Task 1 ArUco checks. Expected TODO until detect_markers is written.

These use generated SIM marker images, not Gazebo, and do not include a
ready-made detector. Run with `bash scripts/python.sh tests/check_aruco.py`.
"""
from pathlib import Path
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "lab7_aruco_exercise", ROOT / "exercises/aruco_reader.py")
EXERCISE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXERCISE)


def main():
    import cv2
    import numpy as np

    def marker_image(marker_id):
        image = cv2.imread(str(ROOT / f"markers/aruco_{marker_id}.png"), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"Missing practice marker {marker_id}")
        return cv2.resize(image, (120, 120), interpolation=cv2.INTER_NEAREST)

    def check(image, expected_ids, label, expected_x):
        try:
            result = EXERCISE.detect_markers(image, "DICT_4X4_50")
        except NotImplementedError:
            print("TODO: implement detect_markers in exercises/aruco_reader.py")
            return False
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise AssertionError(f"{label}: expected (ids, corners)")
        ids, corners = result
        if not isinstance(ids, list) or any(type(i) is not int for i in ids):
            raise AssertionError(f"{label}: IDs must be a list of Python integers")
        if not isinstance(corners, list) or len(corners) != len(ids):
            raise AssertionError(f"{label}: corners must align with IDs")
        if ids != expected_ids:
            raise AssertionError(f"{label}: expected {expected_ids}, got {ids}")
        for value in corners:
            if not isinstance(value, np.ndarray) or value.shape != (1, 4, 2):
                raise AssertionError(f"{label}: each corner array must have shape (1, 4, 2)")
        for marker_id, value in zip(ids, corners):
            if not np.isclose(value[0, :, 0].mean(), expected_x[marker_id], atol=5):
                raise AssertionError(f"{label}: corners do not belong to marker {marker_id}")
        print(f"PASS: {label}")
        return True

    canvas = np.full((320, 640, 3), 255, dtype=np.uint8)

    tests = []
    tests.append((canvas.copy(), [], "no marker clears the result", {}))

    single = canvas.copy()
    single[45:165, 170:290] = cv2.cvtColor(marker_image(23), cv2.COLOR_GRAY2BGR)
    tests.append((single, [23], "single ID 23", {23: 230}))

    multi = canvas.copy()
    for x, marker_id in ((20, 42), (235, 7), (450, 23)):
        multi[45:165, x:x+120] = cv2.cvtColor(marker_image(marker_id), cv2.COLOR_GRAY2BGR)
    tests.append((multi, [7, 23, 42], "three IDs sorted with matching corners",
                  {7: 295, 23: 510, 42: 80}))

    # All IDs in the selected dictionary should decode, even when not mapped
    # to a fruit. Generate an ID that is not one of the three SIM labels.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, "drawMarker", None) or cv2.aruco.generateImageMarker
    unknown = np.full((320, 640, 3), 255, dtype=np.uint8)
    unknown[45:165, 170:290] = cv2.cvtColor(draw(dictionary, 13, 120), cv2.COLOR_GRAY2BGR)
    tests.append((unknown, [13], "unknown-but-decodable ID 13", {13: 230}))

    try:
        for case in tests:
            if not check(*case):
                return 1
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print("PASS: all four Task 1 ArUco checks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
