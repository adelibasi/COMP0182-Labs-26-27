"""ArUco detection and observation publishing state."""


class ArucoReader:
    """Read the configured dictionary using OpenCV 4.5.x or newer."""

    def __init__(self, task_config):
        import cv2  # rusty-linter: ignore

        if not hasattr(cv2, "aruco"):
            raise ValueError("OpenCV has no ArUco module; use the Lab 7 apt dependencies")

        name = task_config.get("marker_dictionary")
        if not isinstance(name, str) or not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
            raise ValueError("Set marker_dictionary to an OpenCV ArUco dictionary name")

        self.cv2 = cv2
        self.dictionary_name = name
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        self.marker_count = len(self.dictionary.bytesList)
        mapping = task_config.get("marker_id_to_fruit")
        if not isinstance(mapping, dict):
            raise ValueError("marker_id_to_fruit must map marker IDs to fruit labels")

        self.marker_map = {}
        for key, label in mapping.items():
            if isinstance(key, bool) or not (isinstance(key, int) or
                                           isinstance(key, str) and key.isdecimal()):
                raise ValueError("Marker IDs must be integers")

            marker_id = int(key)
            if not 0 <= marker_id < self.marker_count or marker_id in self.marker_map:
                raise ValueError(f"Invalid or repeated marker ID: {key}")

            if not isinstance(label, str) or not label.strip():
                raise ValueError(f"Marker {marker_id} needs a nonempty fruit label")

            self.marker_map[marker_id] = label.strip()

        aruco = cv2.aruco
        self.parameters = (aruco.DetectorParameters_create()
                           if hasattr(aruco, "DetectorParameters_create")
                           else aruco.DetectorParameters())
        self.detector = (aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(aruco, "ArucoDetector") else None)

    def detect(self, image):
        """Return aligned ID, label and corner lists; unknown labels are None."""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(image)
        else:
            corners, ids, _ = self.cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=self.parameters)

        if ids is None:
            return [], [], []

        order = sorted(range(len(ids)), key=lambda i: int(ids[i][0]))
        marker_ids = [int(ids[i][0]) for i in order]

        return (marker_ids, [self.marker_map.get(i) for i in marker_ids],
                [corners[i] for i in order])

    def annotate(self, image, ids, labels, corners):
        """Draw only image detections; marker IDs do not establish robot pose."""
        import numpy as np  # rusty-linter: ignore

        result = image.copy()
        if ids:
            self.cv2.aruco.drawDetectedMarkers(
                result, corners, np.asarray(ids, dtype=np.int32).reshape(-1, 1))

        for marker_id, label, points in zip(ids, labels, corners):
            x, y = points.reshape(4, 2).min(axis=0)
            self.cv2.putText(result, f"{marker_id}: {label or 'unknown'}",
                             (max(0, int(x)), max(18, int(y) - 8)),
                             self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)

        return result


class ObservationTracker:
    """Clear old detections using receipt time and the source image's clock.

    now_mono is monotonic wall time, including while Gazebo is paused.
    now_ns and source_stamp_ns must share a ROS clock. A physical bridge may
    stamp at decode time; this class cannot recover sensor acquisition time.
    Duplicate/out-of-order stamps never renew an observation. A ROS clock
    reset clears the previous run's observation and permits new smaller stamps.
    """

    def __init__(self, stale_after=2.0, future_tolerance=0.5):
        self.stale_after = stale_after
        self.future_tolerance = future_tolerance
        self.received_count = 0
        self._last_clock = self._last_stamp = self._arrival = None
        self._last_frame = ""
        self._record = dict(source_stamp_ns=None, frame_id="", image_width=0,
                            image_height=0, ids=[], labels=[])
        self.status, self.reason = "STALE", "waiting for an image"

    def _clear(self, reason):
        self.status, self.reason = "STALE", reason
        self._record["ids"], self._record["labels"] = [], []

    def _clock(self, now_ns):
        if self._last_clock is not None and now_ns < self._last_clock:
            self._last_stamp = self._arrival = None
            self._last_frame = ""
            self._record.update(source_stamp_ns=None, frame_id="",
                                image_width=0, image_height=0)
            self._clear("ROS clock reset; waiting for a new image")

        self._last_clock = now_ns

    def _stamp_error(self, now_ns):
        stamp = self._record["source_stamp_ns"]
        if stamp is None or stamp <= 0:
            return "image timestamp is missing or zero"

        if now_ns <= 0:
            return "ROS clock has not started"

        age = (now_ns - stamp) / 1e9
        if age < -self.future_tolerance:
            return "image timestamp is ahead of this ROS clock"

        if age > self.stale_after:
            return f"source image is older than {self.stale_after:g} seconds"

        return None

    def update(self, *, source_stamp_ns, frame_id, width, height, ids, labels,
               now_mono, now_ns, error=None):
        """Record one input image; return True only for a usable new image."""
        self.received_count += 1
        self._clock(now_ns)
        if (error is None and self._last_stamp is not None and
                frame_id == self._last_frame and source_stamp_ns <= self._last_stamp):
            return False

        self._record.update(source_stamp_ns=source_stamp_ns, frame_id=frame_id,
                            image_width=width, image_height=height)
        self._arrival = now_mono
        error = error or ("image frame_id is empty" if not frame_id else None)
        error = error or ("image dimensions are empty" if width <= 0 or height <= 0 else None)
        error = error or self._stamp_error(now_ns)
        if error:
            self._clear(error)

            return False

        if len(ids) != len(labels):
            raise ValueError("Every marker ID must have one label (or None)")

        self._record["ids"], self._record["labels"] = list(ids), list(labels)
        self._last_stamp, self._last_frame = source_stamp_ns, frame_id
        self.status = "LIVE" if ids else "NO_MARKER"
        self.reason = "" if ids else "no marker in this image"

        return True

    def snapshot(self, now_mono, now_ns):
        """Return JSON-ready evidence; STALE always has empty IDs and labels."""
        self._clock(now_ns)
        if self.status != "STALE":
            if self._arrival is None or now_mono - self._arrival > self.stale_after:
                self._clear(f"no new image for {self.stale_after:g} seconds")
            else:
                error = self._stamp_error(now_ns)
                if error:
                    self._clear(error)

        stamp = self._record["source_stamp_ns"]
        age = (now_ns - stamp) / 1e9 if stamp and now_ns > 0 else None

        return dict(self._record, ids=list(self._record["ids"]),
                    labels=list(self._record["labels"]), status=self.status,
                    reason=self.reason, received_count=self.received_count,
                    received_age_s=(round(max(0, now_mono - self._arrival), 3)
                                    if self._arrival is not None else None),
                    source_age_s=round(age, 3) if age is not None else None)
