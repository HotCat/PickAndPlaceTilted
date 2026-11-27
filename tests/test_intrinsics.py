"""Synthetic ground-truth intrinsics and native-dataset identity/quality checks."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from pcbvision.intrinsics import (IntrinsicSession, capture_identity, detect_corners,
                                  coverage, object_grid, observation_metrics, DETECTOR_VERSION)


def payload(frame=None):
    if frame is None:
        frame = np.full((960, 1280), 160, np.uint8)
    return {'frame': frame, 'metadata': {'backend': 'mindvision', 'serial': 'camera-1',
            'parameters': {'resolution_id': 0, 'reverse_x': 0, 'reverse_y': 0,
                           'acquisition_geometry': {'iWidth': frame.shape[1], 'iHeight': frame.shape[0],
                                                    'iHOffsetFOV': 0, 'uBinSumMode': 0}}}}


def observations(count=24, flat=False):
    k = np.array([[1000., 0, 640], [0, 1010, 480], [0, 0, 1.]])
    d = np.zeros(5) if flat else np.array([-.09, .025, .001, -.0008, -.006])
    obj = object_grid((11, 8), 3.)
    rng = np.random.default_rng(19)
    for index in range(count):
        rvec = np.array([0., 0., rng.uniform(-.2, .2)]) if flat else rng.uniform([-.55, -.5, -.3], [.55, .5, .3])
        rot = cv2.Rodrigues(rvec)[0]
        z = rng.uniform(72, 105)
        center_px = np.array([280+(index%5)*175, 235+(index%4)*160])
        center_cam = np.array([(center_px[0]-640)*z/1000, (center_px[1]-480)*z/1010, z])
        translation = center_cam-rot@obj.mean(axis=0)
        corners = cv2.projectPoints(obj, rvec, translation, k, d)[0]
        if not flat:
            corners += rng.normal(0, .025, corners.shape)
        yield k, d, corners.astype(np.float32)


def test_detects_rendered_board_and_refines_native_pixels():
    # 12x9 squares produce 11x8 interior corners, not 12x9 corners.
    image = np.full((1050, 1400), 210, np.uint8)
    for row in range(9):
        for col in range(12):
            cv2.rectangle(image, (150+col*75, 140+row*75), (150+(col+1)*75, 140+(row+1)*75),
                          0 if (row+col)%2 == 0 else 255, -1)
    corners = detect_corners(image, (11, 8))
    assert corners.shape == (88, 1, 2)
    assert observation_metrics(image, corners, (11, 8))['min_spacing_px'] == pytest.approx(75, abs=.2)
    np.testing.assert_allclose(np.sort(corners[:, 0, 0].reshape(8, 11).ravel())[::8],
                               np.repeat(np.arange(225, 976, 75), 8)[::8], atol=.6)


def test_recovers_known_intrinsics_and_persists_per_view_diagnostics(tmp_path):
    session = IntrinsicSession(tmp_path/'session')
    for k, d, points in observations():
        session.add(payload(), 'lens fixed at mark A', points)
    result = session.solve()
    np.testing.assert_allclose(np.array(result['K'])[:2, :], k[:2, :], atol=2.)
    np.testing.assert_allclose(result['D'][:4], d[:4], atol=.006)
    assert result['rms_px'] < .05
    assert len(result['views']) == 24
    assert result['normal_spread_deg'] > 30
    assert len(result['coverage']['cells']) >= 5
    assert result['identity']['serial'] == 'camera-1'
    assert result['square_size_mm'] == 3.
    loaded = IntrinsicSession.load(session.directory)
    assert len(loaded.samples) == 24
    assert loaded.data['result']['rms_px'] == result['rms_px']
    loaded.export(tmp_path/'result.xml')
    fs = cv2.FileStorage(str(tmp_path/'result.xml'), cv2.FILE_STORAGE_READ)
    np.testing.assert_allclose(fs.getNode('K').mat(), result['K'])
    fs.release()
    assert json.loads((tmp_path/'result.json').read_text())['identity'] == result['identity']
    loaded.set_enabled(0, False)
    assert 'result' not in loaded.data
    assert not (loaded.directory/'intrinsics.json').exists()
    with pytest.raises(ValueError, match='Fit'):
        loaded.export(tmp_path/'invalid.json')


def test_rejects_front_parallel_views_even_when_corner_locations_are_different(tmp_path):
    session = IntrinsicSession(tmp_path)
    for _, _, points in observations(12, flat=True):
        session.add(payload(), 'fixed', points)
    with pytest.raises(ValueError, match='tilt|underconstrained'):
        session.solve()


@pytest.mark.parametrize('change', ['serial', 'resolution', 'roi', 'binning', 'mirror', 'lens'])
def test_does_not_mix_camera_geometry_or_lens_setups(tmp_path, change):
    session = IntrinsicSession(tmp_path)
    points = next(observations())[2]
    session.add(payload(), 'fixed', points)
    new = payload()
    lens = 'fixed'
    if change == 'serial':
        new['metadata']['serial'] = 'another-camera'
    elif change == 'resolution':
        new['metadata']['parameters']['resolution_id'] = 4
    elif change == 'roi':
        new['metadata']['parameters']['acquisition_geometry']['iHOffsetFOV'] = 8
    elif change == 'binning':
        new['metadata']['parameters']['acquisition_geometry']['uBinSumMode'] = 1
    elif change == 'mirror':
        new['metadata']['parameters']['reverse_x'] = 1
    else:
        lens = 'focus changed'
    with pytest.raises(ValueError, match='changed'):
        session.add(new, lens, points)
    assert len(session.samples) == 1


def test_rejects_duplicate_reversed_indexing_and_requires_ten_views(tmp_path):
    session = IntrinsicSession(tmp_path)
    points = next(observations())[2]
    session.add(payload(), 'fixed', points)
    with pytest.raises(ValueError, match='duplicates'):
        session.add(payload(), 'fixed', points[::-1])
    with pytest.raises(ValueError, match='10'):
        session.solve()


def test_sample_write_failure_keeps_existing_manifest_and_images(tmp_path, monkeypatch):
    session = IntrinsicSession(tmp_path)
    views = observations(2)
    points = next(views)[2]
    session.add(payload(), 'fixed', points)
    manifest = session.manifest.read_bytes()
    monkeypatch.setattr(session, 'save', lambda: (_ for _ in ()).throw(OSError('disk full')))
    with pytest.raises(OSError, match='disk full'):
        session.add(payload(), 'fixed', next(views)[2])
    assert len(session.samples) == 1
    assert len(list(tmp_path.glob('*.png'))) == 1
    assert session.manifest.read_bytes() == manifest


def test_rejects_missing_camera_serial(tmp_path):
    data = payload()
    data['metadata']['serial'] = ''
    with pytest.raises(ValueError, match='serial'):
        capture_identity(data, 'fixed')


def test_sb_subpixel_coordinates_survive_fallback_resize_without_gradient_refinement(monkeypatch):
    # Force the native attempt to fail, then return known fractional SB corners
    # at the next scale. No cornerSubPix may modify these estimates afterward.
    image = np.zeros((1501, 2001), np.uint8)
    points = np.mgrid[:11, :8].T.reshape(-1, 1, 2).astype(np.float32)*30+[100.25, 90.75]
    calls = []
    def sb(frame, board, flags):
        calls.append(frame.shape)
        return (False, None) if len(calls) == 1 else (True, points.astype(np.float32).copy())
    monkeypatch.setattr(cv2, 'findChessboardCornersSB', sb)
    def forbidden(*_, **__):
        raise AssertionError('SB is already subpixel; extra cornerSubPix regresses real-camera accuracy')
    monkeypatch.setattr(cv2, 'cornerSubPix', forbidden)
    result = detect_corners(image, (11, 8))
    expected = (points+.5)*[image.shape[1]/calls[1][1], image.shape[0]/calls[1][0]]-.5
    np.testing.assert_allclose(result, expected, atol=.0001)


def test_large_board_coverage_uses_corner_regions_not_center_locations(tmp_path):
    session = IntrinsicSession(tmp_path)
    obj = object_grid((11, 8), 3.)
    k = np.array([[1000., 0, 640], [0, 1000, 480], [0, 0, 1.]])
    rng = np.random.default_rng(73)
    for _ in range(100):
        rvec = rng.uniform([-.22, -.22, -.08], [.22, .22, .08])
        rot = cv2.Rodrigues(rvec)[0]
        trans = np.array([0., 0., 28.])-rot@obj.mean(axis=0)
        points = cv2.projectPoints(obj, rvec, trans, k, np.zeros(5))[0]
        if np.any(points.reshape(-1, 2).min(axis=0) < 10) or np.any(points.reshape(-1, 2).max(axis=0) > [1270, 950]):
            continue
        session.add(payload(), 'fixed', points)
        if len(session.samples) == 20:
            break
    assert len(session.samples) == 20
    report = coverage(session.active, (1280, 960))
    assert len(report['cells']) == 1  # all board centroids stay in center
    assert len(report['corner_cells']) == 9
    assert report['image_hull_fraction'] > .7
    result = session.solve()
    assert not any('coverage' in warning for warning in result['warnings'])


def test_reprocess_preserves_original_images_manifest_and_exclusions(tmp_path):
    image = np.full((1050, 1400), 210, np.uint8)
    for row in range(9):
        for col in range(12):
            image[140+row*75:140+(row+1)*75, 150+col*75:150+(col+1)*75] = 0 if (row+col)%2 == 0 else 255
    source = IntrinsicSession(tmp_path/'source')
    source.add(payload(image), 'fixed')
    source.set_enabled(0, False)
    source.data['samples'][0].pop('detector')
    source.data['result'] = {'rms_px': 1.05}
    source.save()
    before = source.manifest.read_bytes()
    image_before = (source.directory/source.samples[0]['image']).read_bytes()
    messages = []
    target = source.reprocess(tmp_path/'corrected', messages.append)
    assert messages
    assert source.manifest.read_bytes() == before
    assert 'result' in source.data and 'result' not in target.data
    assert target.data['reprocessed_from']['rms_px'] == 1.05
    assert target.samples[0]['enabled'] is False
    assert target.samples[0]['detector']['version'] == DETECTOR_VERSION
    assert (target.directory/target.samples[0]['image']).read_bytes() == image_before
    assert IntrinsicSession.load(target.directory).samples[0]['detector']['extra_cornerSubPix'] is False
    with pytest.raises(FileExistsError):
        source.reprocess(tmp_path/'corrected')


def test_reprocess_detection_failure_leaves_original_and_removes_partial_copy(tmp_path, monkeypatch):
    source = IntrinsicSession(tmp_path/'source')
    source.add(payload(), 'fixed', next(observations())[2])
    original = source.manifest.read_bytes()
    def fail(*_):
        raise ValueError('No chessboard')
    monkeypatch.setattr('pcbvision.intrinsics._detect_observation', fail)
    with pytest.raises(ValueError, match='No chessboard'):
        source.reprocess(tmp_path/'corrected')
    assert source.manifest.read_bytes() == original
    assert not (tmp_path/'corrected').exists()
