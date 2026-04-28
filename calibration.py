"""
Camera calibration and rectification for the Pick & Place vision guide.

Provides:
  - Intrinsic calibration (K, D) from chessboard images
  - Pose estimation (R, tvec) from a single chessboard image + known intrinsics
  - Frame rectification (undistort + fronto-parallel warp) with cached maps
  - Persistence via OpenCV FileStorage XML
"""

import os
import cv2
import numpy as np


# ── Chessboard detection ─────────────────────────────────────────────────

def detect_chessboard(image, board_size=(11, 8)):
    """Detect chessboard corners and refine to sub-pixel accuracy.

    Returns (found, corners). corners is None when found is False.
    """
    if image.ndim == 2:
        gray = image
    elif image.shape[2] == 1:
        gray = image[:, :, 0]
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    found, corners = cv2.findChessboardCorners(
        gray, board_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if found:
        cv2.cornerSubPix(gray, corners, (11, 8), (-1, -1), criteria)
    return found, corners


# ── Intrinsic calibration ────────────────────────────────────────────────

def calibrate_camera(image_paths, board_size=(11, 8), square_size=5.0):
    """Run full intrinsic calibration from a list of chessboard image paths.

    Returns (rms, K, D) on success, or None if fewer than 5 valid images.
    K = 3x3 camera matrix, D = distortion coefficients.
    """
    objp = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp[:, :2] *= square_size

    object_points = []
    image_points = []
    image_size = None

    for path in image_paths:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        image_size = img.shape[::-1]  # (width, height)

        found, corners = detect_chessboard(img, board_size)
        if not found:
            continue

        image_points.append(corners)
        object_points.append(objp)

    if len(image_points) < 5:
        return None

    rms, K, D, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None,
    )
    return rms, K, D


# ── Pose estimation ─────────────────────────────────────────────────────

def estimate_pose(image, K, D, board_size=(11, 8), square_size=5.0):
    """Estimate camera pose (R, tvec) relative to the chessboard plane.

    Returns (R, tvec) on success, or None if chessboard not found.
    R = 3x3 rotation matrix, tvec = 3x1 translation vector.
    """
    objp = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp[:, :2] *= square_size

    found, corners = detect_chessboard(image, board_size)
    if not found:
        return None

    success, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec


# ── Rectifier (cached maps) ─────────────────────────────────────────────

class Rectifier:
    """Pre-computes undistort maps + homography so per-frame work is minimal."""

    def __init__(self):
        self._map1 = None
        self._map2 = None
        self._Hfinal = None
        self._output_size = None

    @property
    def ready(self) -> bool:
        return self._Hfinal is not None

    def setup(self, K, D, R, tvec, image_size, scale=15.0):
        """Pre-compute all rectification maps.

        Parameters
        ----------
        K, D : camera matrix and distortion coefficients
        R    : 3x3 rotation matrix (from solvePnP)
        tvec : 3x1 translation vector (from solvePnP)
        image_size : (width, height) of input frames
        scale : pixels per world-unit (mm) in the rectified output
        """
        w, h = image_size

        # Undistort maps (equivalent to cv2.undistort per frame)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            K, D, np.eye(3), K, (w, h), cv2.CV_16SC2,
        )

        # Homography: plane-to-image  H = K [r1 | r2 | t]
        r1 = R[:, 0:1]
        r2 = R[:, 1:2]
        Rt = np.hstack([r1, r2, tvec.reshape(3, 1)])
        H = K.astype(np.float64) @ Rt.astype(np.float64)
        Hinv = np.linalg.inv(H)

        Hscale = np.array([
            [scale, 0, w / 2.0],
            [0, scale, h / 2.0],
            [0, 0, 1],
        ], dtype=np.float64)

        self._Hfinal = Hscale @ Hinv
        self._output_size = (w, h)

    def rectify(self, frame):
        """Apply undistort + fronto-parallel warp to a BGR frame."""
        if self._Hfinal is None:
            return frame
        undistorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
        return cv2.warpPerspective(
            undistorted, self._Hfinal, self._output_size, cv2.INTER_LINEAR,
        )


# ── Persistence (OpenCV FileStorage XML) ─────────────────────────────────

def save_calibration(filepath, K, D, R=None, tvec=None):
    """Write calibration parameters to an XML file."""
    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
    fs.write("K", K)
    fs.write("D", D)
    if R is not None:
        fs.write("R", R)
    if tvec is not None:
        fs.write("tvec", tvec)
    fs.release()


def load_calibration(filepath):
    """Read calibration parameters from an XML file.

    Returns (K, D, R, tvec).  R and tvec are None when not present.
    Returns all None when the file cannot be read.
    """
    if not os.path.exists(filepath):
        return None, None, None, None

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    try:
        K = fs.getNode("K").mat()
        D = fs.getNode("D").mat()
        R_node = fs.getNode("R")
        t_node = fs.getNode("tvec")
        R = R_node.mat() if not R_node.empty() else None
        tvec = t_node.mat() if not t_node.empty() else None
    finally:
        fs.release()

    return K, D, R, tvec
