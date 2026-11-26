"""Native-pixel chessboard intrinsics, with camera/mode-bound, resumable datasets.

This module knows nothing about robots. K maps camera rays to *raw SDK image*
coordinates; distortion uses OpenCV's five-coefficient Brown model. Samples from
another serial, ROI/binning preset, mirror state or lens setup cannot be mixed.
The durable result retains all per-view residuals instead of hiding outliers.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import uuid

import cv2
import numpy as np

from launcher.config import atomic_json

MIN_VIEWS = 10
RECOMMENDED_VIEWS = 20


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def calibration_root():
    return Path(os.environ.get('XDG_DATA_HOME', Path.home()/'.local/share'))/'pickplace-workbench/camera-calibration'


def object_grid(board_size, square_mm):
    cols, rows = board_size
    if not (3 <= cols <= 40 and 3 <= rows <= 40) or not np.isfinite(square_mm) or square_mm <= 0:
        raise ValueError('Invalid inner-corner counts or square size / 内角点数或方格尺寸无效')
    points = np.zeros((cols*rows, 3), np.float32)
    points[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2)*square_mm
    return points


def gray_image(image):
    if image is None or image.dtype != np.uint8:
        raise ValueError('Expected an 8-bit native camera image')
    if image.ndim == 2:
        return image
    if image.ndim == 3 and image.shape[2] == 1:
        return image[:, :, 0]
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def detect_corners(image, board_size):
    """Detect cheaply at limited size, refine in full native image coordinates."""
    gray = gray_image(image)
    scale = min(1., 1600/max(gray.shape))
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else gray
    found, corners = cv2.findChessboardCornersSB(small, tuple(board_size), cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise ValueError('Full chessboard not detected. Keep all corners visible, focused and evenly lit. / 未检测到完整棋盘，请检查尺寸、清晰度及遮挡。')
    # Use the actual resize factors (OpenCV rounds the resized dimensions).
    corners[:, :, 0] *= gray.shape[1]/small.shape[1]
    corners[:, :, 1] *= gray.shape[0]/small.shape[0]
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                              (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    return corners.astype(np.float32)


def observation_metrics(image, corners, board_size):
    h, w = image.shape[:2]
    grid = corners.reshape(board_size[1], board_size[0], 2)
    spacing = np.concatenate((np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel(),
                              np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel()))
    points = corners.reshape(-1, 2)
    if not np.all(np.isfinite(points)) or points[:, 0].min() < 8 or points[:, 1].min() < 8 or points[:, 0].max() > w-8 or points[:, 1].max() > h-8:
        raise ValueError('Corners touch the image border; move the board fully inside / 棋盘需完整位于图像内')
    if spacing.min() < 8:
        raise ValueError('Chessboard is too small/foreshortened; increase its image size / 棋盘过小或倾斜过大')
    hull = cv2.convexHull(points)
    area = cv2.contourArea(hull)/(w*h)
    center = points.mean(axis=0)/[w, h]
    x0, y0 = np.floor(points.min(axis=0)).astype(int)
    x1, y1 = np.ceil(points.max(axis=0)).astype(int)
    sharpness = cv2.Laplacian(gray_image(image)[y0:y1, x0:x1], cv2.CV_64F).var()
    return {'center': center.tolist(), 'area_fraction': float(area),
            'min_spacing_px': float(spacing.min()), 'sharpness': float(sharpness)}


def capture_identity(payload, lens_setup):
    image, metadata = payload['frame'], payload['metadata']
    params = metadata['parameters']
    if not metadata.get('serial'):
        raise ValueError('A camera serial number is required to bind this calibration / 缺少相机序列号')
    if not lens_setup.strip():
        raise ValueError('Describe the fixed lens/focus setup / 请填写固定镜头及对焦设置')
    return {'backend': metadata['backend'], 'serial': metadata['serial'],
            'image_size': [image.shape[1], image.shape[0]],
            'acquisition_geometry': params.get('acquisition_geometry'),
            'resolution_id': params.get('resolution_id'),
            'reverse_x': params.get('reverse_x'), 'reverse_y': params.get('reverse_y'),
            'lens_setup': lens_setup.strip()}


def coverage(samples, image_size):
    if not samples:
        return {'cells': [], 'image_hull_fraction': 0., 'center_span': [0., 0.]}
    w, h = image_size
    corners = np.concatenate([np.asarray(s['corners'], np.float32).reshape(-1, 2) for s in samples])
    centers = np.array([s['metrics']['center'] for s in samples])
    cells = sorted({min(2, int(y*3))*3+min(2, int(x*3)) for x, y in centers})
    return {'cells': cells, 'image_hull_fraction': float(cv2.contourArea(cv2.convexHull(corners))/(w*h)),
            'center_span': np.ptp(centers, axis=0).tolist()}


class IntrinsicSession:
    def __init__(self, directory, board_size=(11, 8), square_mm=3.):
        object_grid(board_size, square_mm)
        self.directory = Path(directory).expanduser().resolve()
        self.data = {'schema_version': 1, 'board_inner_corners': list(board_size),
                     'square_size_mm': float(square_mm), 'created_at': utc_now(),
                     'identity': None, 'samples': []}

    @property
    def board_size(self):
        return tuple(self.data['board_inner_corners'])

    @property
    def samples(self):
        return self.data['samples']

    @property
    def active(self):
        return [s for s in self.samples if s.get('enabled', True)]

    @property
    def manifest(self):
        return self.directory/'session.json'

    def save(self):
        atomic_json(self.manifest, self.data)

    @classmethod
    def load(cls, directory):
        directory = Path(directory)
        data = json.loads((directory/'session.json').read_text())
        if data.get('schema_version') != 1 or not isinstance(data.get('samples'), list):
            raise ValueError('Unsupported intrinsic session / 不支持的内参采集文件')
        session = cls(directory, data['board_inner_corners'], data['square_size_mm'])
        identity = data.get('identity')
        for sample in data['samples']:
            filename = sample['image']
            if Path(filename).name != filename or not (directory/filename).is_file():
                raise ValueError('Missing or invalid calibration sample image')
            points = np.asarray(sample['corners'])
            if points.shape != (np.prod(session.board_size), 2) or not np.all(np.isfinite(points)):
                raise ValueError('Invalid saved corner coordinates')
            image = cv2.imread(str(directory/filename), cv2.IMREAD_GRAYSCALE)
            if image is None or identity is None or list(image.shape[::-1]) != identity['image_size']:
                raise ValueError('Sample dimensions do not match the saved camera mode')
        session.data = data
        return session

    def add(self, payload, lens_setup, corners=None):
        frame = payload['frame']
        identity = capture_identity(payload, lens_setup)
        if self.data['identity'] is not None and identity != self.data['identity']:
            raise ValueError('Camera, resolution/ROI/binning, mirror or lens setup changed. Start a new session. / 相机采集模式或镜头设置已改变，请新建采集。')
        if len(self.samples) >= 100:
            raise ValueError('100-view limit; start a new session')
        points = detect_corners(frame, self.board_size) if corners is None else np.asarray(corners, np.float32).reshape(-1, 1, 2)
        if points.shape != (np.prod(self.board_size), 1, 2):
            raise ValueError('Wrong corner count')
        metrics = observation_metrics(frame, points, self.board_size)
        # An unmarked board can be indexed in either direction. Ignore that
        # reversal when rejecting repeated views of an unmoved camera/target.
        flat = points.reshape(-1, 2)
        for sample in self.active:
            previous = np.asarray(sample['corners'])
            distance = min(np.sqrt(np.mean(np.sum((flat-previous)**2, axis=1))),
                           np.sqrt(np.mean(np.sum((flat[::-1]-previous)**2, axis=1))))
            if distance < max(4., .003*max(frame.shape[:2])):
                raise ValueError('This view duplicates an existing sample. Change board position, distance or tilt. / 视角重复，请改变棋盘位置、距离或倾角。')
        filename = f'view-{uuid.uuid4().hex[:12]}.png'
        self.directory.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.directory/filename), frame):
            raise OSError('Could not save native calibration image')
        sample = {'image': filename, 'captured_at': utc_now(), 'corners': flat.tolist(),
                  'metrics': metrics, 'enabled': True, 'camera_settings': payload['metadata']['parameters']}
        old_identity = self.data['identity']
        self.data['identity'] = identity
        self.samples.append(sample)
        # Any dataset edit invalidates its previous result until refitted.
        old_result = self.data.pop('result', None)
        try:
            self.save()
        except Exception:
            self.samples.pop()
            self.data['identity'] = old_identity
            if old_result is not None:
                self.data['result'] = old_result
            (self.directory/filename).unlink(missing_ok=True)
            raise
        (self.directory/'intrinsics.json').unlink(missing_ok=True)
        return sample, points

    def set_enabled(self, index, enabled):
        previous = self.samples[index].get('enabled', True)
        old_result = self.data.get('result')
        self.samples[index]['enabled'] = bool(enabled)
        self.data.pop('result', None)
        try:
            self.save()
        except Exception:
            self.samples[index]['enabled'] = previous
            if old_result is not None:
                self.data['result'] = old_result
            raise
        (self.directory/'intrinsics.json').unlink(missing_ok=True)

    def solve(self):
        samples = self.active
        if len(samples) < MIN_VIEWS:
            raise ValueError(f'Collect at least {MIN_VIEWS} different views (20–30 recommended) / 至少采集 {MIN_VIEWS} 个不同视角')
        identity = self.data['identity']
        size = tuple(identity['image_size'])
        obj = object_grid(self.board_size, self.data['square_size_mm'])
        image_points = [np.asarray(s['corners'], np.float32).reshape(-1, 1, 2) for s in samples]
        # A planar target observed with only translations/in-plane rotation is
        # degenerate for intrinsics. Test the normalized Zhang constraint rank
        # before trusting a low reprojection error from such a data set.
        rows = []
        norm = np.array([[1/size[0], 0, -.5], [0, 1/size[1], -.5], [0, 0, 1.]])
        def vij(h, i, j):
            return np.array([h[0,i]*h[0,j], h[0,i]*h[1,j]+h[1,i]*h[0,j],
                             h[1,i]*h[1,j], h[2,i]*h[0,j]+h[0,i]*h[2,j],
                             h[2,i]*h[1,j]+h[1,i]*h[2,j], h[2,i]*h[2,j]])
        for points in image_points:
            h, _ = cv2.findHomography(obj[:, :2], points)
            h = norm@h
            a, b = vij(h, 0, 1), vij(h, 0, 0)-vij(h, 1, 1)
            rows += [a/max(np.linalg.norm(a), 1e-15), b/max(np.linalg.norm(b), 1e-15)]
        singular = np.linalg.svd(np.asarray(rows), compute_uv=False)
        if singular[-2]/singular[0] < 1e-5:
            raise ValueError('Views lack out-of-plane tilt; intrinsics are underconstrained. Tilt about both board axes. / 缺少出平面倾角，请绕两个方向倾斜棋盘。')
        rms, k, d, rotations, translations, std_k, _, per_view = cv2.calibrateCameraExtended(
            [obj.copy() for _ in samples], image_points, size, None, None, flags=0,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8))
        if not np.isfinite(rms) or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or min(k[0,0], k[1,1]) <= 0:
            raise ValueError('Calibration produced invalid intrinsics; inspect the samples')
        # Lens parameters should not be accepted solely because training RMS is
        # low. Report coverage, orientation diversity and parameter uncertainty.
        normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rotations])
        angle = float(np.degrees(np.arccos(np.clip((normals@normals.T).min(), -1., 1.))))
        report = coverage(samples, size)
        warnings = []
        if angle < 12:
            raise ValueError('Insufficient tilt diversity (<12° fitted normal spread). Add views tilted in both directions. / 倾角变化不足，请补充不同倾角。')
        tilt_axes = np.linalg.svd(normals[:, :2]-normals[:, :2].mean(axis=0), compute_uv=False)/np.sqrt(len(normals))
        if np.degrees(tilt_axes[-1]) < 3:
            warnings.append('Limited second-axis tilt; add views tilted about the other board axis / 第二方向倾角不足')
        if len(samples) < RECOMMENDED_VIEWS:
            warnings.append('Fewer than 20 views / 少于20张')
        if len(report['cells']) < 5 or min(report['center_span']) < .3 or report['image_hull_fraction'] < .55:
            warnings.append('Limited image coverage: add center, edge and corner views / 需补充图像边缘和角落视图')
        if rms > .5 or float(per_view.max()) > 1.:
            warnings.append('Large pixel residuals: inspect high-error views / 重投影误差偏大，请检查高误差样本')
        if not (0 < k[0,2] < size[0] and 0 < k[1,2] < size[1]) or not .1 < k[0,0]/size[0] < 10 or not .1 < k[1,1]/size[1] < 10:
            warnings.append('Unusual focal length/principal point; check target size and view diversity / 焦距或主点异常')
        relative_std = np.asarray(std_k).ravel()[:2]/[k[0,0], k[1,1]]
        if not np.all(np.isfinite(relative_std)) or relative_std.max() > .05:
            warnings.append('Focal-length uncertainty exceeds 5%; add diverse views / 焦距不确定度大于5%')
        result = {'schema_version': 1, 'kind': 'camera_intrinsics', 'created_at': utc_now(),
                  'opencv_version': cv2.__version__, 'model': 'opencv_pinhole_brown5',
                  'distortion_order': ['k1', 'k2', 'p1', 'p2', 'k3'],
                  'identity': identity, 'board_inner_corners': list(self.board_size),
                  'square_size_mm': self.data['square_size_mm'], 'image_size': list(size),
                  'K': k.tolist(), 'D': d.ravel().tolist(), 'rms_px': float(rms),
                  'intrinsic_std': std_k.ravel().tolist(), 'normal_spread_deg': angle,
                  'coverage': report, 'warnings': warnings,
                  'status': 'needs_review' if warnings else 'fit_complete_requires_independent_validation',
                  'views': [{'image': s['image'], 'rms_px': float(error),
                             'board_to_camera_rvec': r.ravel().tolist(), 'board_to_camera_t_mm': t.ravel().tolist()}
                            for s, error, r, t in zip(samples, per_view.ravel(), rotations, translations)],
                  'notes': 'Training residuals are pixels, not TCP/mm accuracy. Not hand-eye calibration. Valid only for this camera, acquisition mode and fixed lens setup.'}
        self.data['result'] = result
        try:
            self.save()
        except Exception:
            self.data.pop('result', None)
            raise
        atomic_json(self.directory/'intrinsics.json', result)
        return result

    def export(self, path):
        result = self.data.get('result')
        if result is None:
            raise ValueError('Fit the current dataset before exporting / 请先计算当前数据集')
        path = Path(path)
        if path.suffix.lower() == '.xml':
            # Compatible with the existing rectification workbench; retain the
            # richer JSON beside XML so acquisition identity is never omitted.
            fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
            if not fs.isOpened():
                raise OSError(f'Cannot write {path}')
            try:
                fs.write('K', np.asarray(result['K']))
                fs.write('D', np.asarray(result['D']))
                fs.write('image_width', result['image_size'][0])
                fs.write('image_height', result['image_size'][1])
                fs.write('rms_px', result['rms_px'])
                fs.write('camera_serial', result['identity']['serial'])
                fs.write('metadata_file', path.with_suffix('.json').name)
            finally:
                fs.release()
            atomic_json(path.with_suffix('.json'), result)
        else:
            atomic_json(path, result)
