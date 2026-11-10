#!/usr/bin/env python3
"""
Calibrate the See3CAM_24CUG (or any V4L2 camera) with a chessboard.

Live mode shows the camera with the detected board drawn on it and captures a view
automatically when the board is held still in a pose unlike the views already taken
(position, size and tilt, as ROS's camera_calibration does). Dots mark every corner
captured so far: fill the whole frame with them, corners and edges especially, since
that is where the lens distorts most. Keys:

  space   capture now (board must be detected)
  c       calibrate (needs --min-views views)
  u       cycle the preview: raw / undistorted with each fitted model
  q, Esc  quit

Calibration fits two models and reports both:
  rational_polynomial  OpenCV pinhole + 8 coefficients (k1 k2 p1 p2 k3 k4 k5 k6)
  equidistant          OpenCV fisheye, 4 coefficients (k1..k4)
The training RMS always favours the model with more parameters, so each model is
also scored on held-out views (5-fold: fit on 4/5, solve each held-out view's pose,
measure its reprojection error). Views whose error is far above the rest (a blurred or
misdetected board) are dropped before the final fit.

Results (ROS camera_info YAML per model, report.txt) go to the session directory and
to camera/calibration/ in the repo. Images are kept in the session directory, so the
calibration can be rerun with --images.

Usage:
  tools/calibrate_camera.py                        # /dev/video0, 1920x1200, 9x6 corners, 55 mm
  tools/calibrate_camera.py --images ~/camera_calib/20261109_153000
  tools/calibrate_camera.py --images ~/camera_calib/20261109_153000 --preview   # + live undistorted view
  tools/calibrate_camera.py --device /dev/video2 --square 0.055 --cols 9 --rows 6
"""

import argparse
import glob
import os
import threading
import time

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Sensor: AR0234, 1920x1200 pixels of 3.0 um. The 1920x1080 mode crops rows, 1280x720 scales that.
SENSOR_SIZE = (1920, 1200)
PIXEL_MM = 0.003

# Results also go into the repo, if the script is running from a checkout of it
REPO_OUT = os.path.join(REPO, 'camera', 'calibration') if os.path.isdir(os.path.join(REPO, 'camera')) else None
SESSION_ROOT = os.path.expanduser('~/camera_calib')

# A new view must differ from every captured one by this much (sum over x, y, size, skew)
MIN_POSE_DIFF = 0.2
# Board counts as still when no corner moved more than this (full-res px) for STILL_FRAMES
STILL_PX = 2.0
STILL_FRAMES = 4
# Ranges of x, y, size, skew that a complete set should span (from ROS camera_calibration)
GOAL_RANGE = {'X': 0.7, 'Y': 0.7, 'Size': 0.4, 'Skew': 0.5}


# ---------------------------------------------------------------- camera

class Camera:
    """Reads frames in a thread so the preview always shows the newest one."""

    def __init__(self, device, width, height, fps):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f'cannot open {device}')
        w, h = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (w, h) != (width, height):
            raise SystemExit(f'{device} gave {w}x{h}, asked for {width}x{height}')
        self.frame, self.seq = None, 0
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.seq = f, self.seq + 1

    def latest(self):
        with self.lock:
            return self.frame, self.seq

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


# ---------------------------------------------------------------- detection

def detect_fast(gray_small, pattern):
    """Quick detection on the downscaled preview frame; None when no board."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray_small, pattern, flags=flags)
    return corners.reshape(-1, 2) if ok else None


def detect_accurate(gray, pattern, guess=None):
    """Full-resolution corners: the sector-based detector, else subpixel-refined guess."""
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:
        return corners.reshape(-1, 2).astype(np.float32)
    if guess is None:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, guess = cv2.findChessboardCorners(gray, pattern, flags=flags)
        if not ok:
            return None
    c = guess.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(gray, c, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01))
    return c.reshape(-1, 2)


def pose_params(corners, pattern, size):
    """x, y, size, skew in [0, 1], as ROS camera_calibration defines them."""
    cols, rows = pattern
    w, h = size
    ul, ur, dr, dl = corners[0], corners[cols - 1], corners[-1], corners[-cols]
    quad = np.array([ul, ur, dr, dl])
    area = 0.5 * abs(np.cross(quad[2] - quad[0], quad[3] - quad[1]))
    border = np.sqrt(area)
    mx, my = corners.mean(axis=0)
    x = min(1.0, max(0.0, (mx - border / 2) / (w - border)))
    y = min(1.0, max(0.0, (my - border / 2) / (h - border)))

    a, b = ul - ur, dr - ur
    ang = np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    skew = min(1.0, 2.0 * abs(np.pi / 2 - ang))
    return np.array([x, y, np.sqrt(area / (w * h)), skew])


# ---------------------------------------------------------------- calibration

def object_points(pattern, square):
    cols, rows = pattern
    g = np.zeros((rows * cols, 3), np.float64)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return g


def fit(model, objp, img_pts, size):
    """Returns (rms, K, D) or None if the solver failed."""
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-10)
    try:
        if model == 'rational_polynomial':
            rms, K, D, _, _ = cv2.calibrateCamera(
                [objp.astype(np.float32)] * len(img_pts), [p.astype(np.float32) for p in img_pts],
                size, None, None, flags=cv2.CALIB_RATIONAL_MODEL, criteria=crit)
            return rms, K, D.ravel()[:8]
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            [objp.reshape(-1, 1, 3)] * len(img_pts), [p.reshape(-1, 1, 2).astype(np.float64) for p in img_pts],
            size, None, None,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW, criteria=crit)
        return rms, K, D.ravel()
    except cv2.error as e:
        print(f'  {model}: solver failed: {str(e).strip().splitlines()[-1]}')
        return None


def view_error(model, K, D, objp, pts):
    """RMS reprojection error (px) of one view after solving its pose with fixed intrinsics."""
    pts = pts.reshape(-1, 1, 2).astype(np.float64)
    if model == 'rational_polynomial':
        ok, rvec, tvec = cv2.solvePnP(objp, pts, K, D)
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    else:
        norm = cv2.fisheye.undistortPoints(pts, K, D)
        ok, rvec, tvec = cv2.solvePnP(objp, norm, np.eye(3), None)
        if ok:  # refine in pixel space through the fisheye projection
            rvec, tvec = refine_fisheye_pose(objp, pts, K, D, rvec, tvec)
        proj, _ = cv2.fisheye.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec, K, D)
    if not ok:
        return np.inf
    return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - pts.reshape(-1, 2)) ** 2, axis=1))))


def refine_fisheye_pose(objp, pts, K, D, rvec, tvec, iters=10):
    """Gauss-Newton on the pose, since solvePnP minimised error in undistorted space."""
    x = np.vstack([rvec, tvec]).ravel()
    for _ in range(iters):
        proj, jac = cv2.fisheye.projectPoints(objp.reshape(-1, 1, 3), x[:3].reshape(3, 1),
                                              x[3:].reshape(3, 1), K, D)
        r = (proj - pts).reshape(-1)
        J = jac[:, 8:14]  # columns: f(2) c(2) k(4) rvec(3) tvec(3) skew(1)
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        x += step
        if np.linalg.norm(step) < 1e-10:
            break
    return x[:3].reshape(3, 1), x[3:].reshape(3, 1)


def crossval(model, objp, img_pts, size, folds=5):
    errs = []
    idx = np.arange(len(img_pts))
    for f in range(folds):
        test = idx[f::folds]
        train = np.setdiff1d(idx, test)
        r = fit(model, objp, [img_pts[i] for i in train], size)
        if r is None:
            return np.inf
        _, K, D = r
        errs += [view_error(model, K, D, objp, img_pts[i]) for i in test]
    errs = np.array(errs)
    return float(np.sqrt(np.mean(errs ** 2)))


def new_camera_matrix(model, K, D, size):
    """
    Projection for the rectified image: one focal length, zoomed so every pixel is valid.

    Not cv2.getOptimalNewCameraMatrix: it scales x and y separately, which squashed the
    rectified image of this lens horizontally to 0.76x.
    """
    w, h = size
    n = 400
    edges = {
        'left': np.stack([np.zeros(n), np.linspace(0, h - 1, n)], 1),
        'right': np.stack([np.full(n, w - 1.0), np.linspace(0, h - 1, n)], 1),
        'top': np.stack([np.linspace(0, w - 1, n), np.zeros(n)], 1),
        'bottom': np.stack([np.linspace(0, w - 1, n), np.full(n, h - 1.0)], 1),
    }
    und = {}
    for k, pts in edges.items():
        pts = pts.reshape(-1, 1, 2)
        if model == 'rational_polynomial':
            crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-12)
            u = cv2.undistortPointsIter(pts, K, D, None, None, crit).reshape(-1, 2)
            und[k] = u[np.isfinite(u).all(axis=1)]  # NaN past where the model folds back
        else:
            und[k] = cv2.fisheye.undistortPoints(pts, K, D).reshape(-1, 2)
    # Largest axis-aligned rectangle (in normalised coordinates) inside the image's outline
    xl, xr = und['left'][:, 0].max(), und['right'][:, 0].min()
    yt, yb = und['top'][:, 1].max(), und['bottom'][:, 1].min()
    f = max((w - 1) / (xr - xl), (h - 1) / (yb - yt))
    cx = (w - 1) / 2 - f * (xl + xr) / 2
    cy = (h - 1) / 2 - f * (yt + yb) / 2
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])


def off_axis_angles(model, K, D, size):
    """
    Degrees off the optical axis at the left, right, top, bottom edge centres and the corners.

    Also returns where the radial curve stops increasing (rational model only; the
    equidistant one is checked over 0-90 deg). Pixels beyond it can't be undistorted.
    """
    w, h = size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pts = {'left': (0, cy), 'right': (w - 1, cy), 'top': (cx, 0), 'bottom': (cx, h - 1),
           'corner': max([(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)],
                         key=lambda p: np.hypot((p[0] - cx) / fx, (p[1] - cy) / fy))}
    if model == 'rational_polynomial':
        k1, k2, _, _, k3, k4, k5, k6 = D
        ru = np.linspace(0, 6, 60001)
        r2 = ru * ru
        rd = ru * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)
        angle = np.degrees(np.arctan(ru))
    else:
        th = np.linspace(0, np.pi / 2, 20001)
        rd = th * (1 + D[0] * th ** 2 + D[1] * th ** 4 + D[2] * th ** 6 + D[3] * th ** 8)
        angle = np.degrees(th)
    dec = np.nonzero(np.diff(rd) <= 0)[0]
    end = dec[0] + 1 if len(dec) else len(rd)
    out = {}
    for k, (u, v) in pts.items():
        r = np.hypot((u - cx) / fx, (v - cy) / fy)
        out[k] = float(np.interp(r, rd[:end], angle[:end])) if r <= rd[end - 1] else None
    return out, float(angle[end - 1]), bool(len(dec))


def undistort_maps(model, K, D, P, size):
    if model == 'rational_polynomial':
        return cv2.initUndistortRectifyMap(K, D, np.eye(3), P, size, cv2.CV_16SC2)
    return cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, size, cv2.CV_16SC2)


def yaml_matrix(name, m):
    m = np.atleast_2d(m)
    data = ', '.join(f'{v:.10g}' for v in m.ravel())
    return f'{name}:\n  rows: {m.shape[0]}\n  cols: {m.shape[1]}\n  data: [{data}]\n'


def write_camera_info(path, name, model, K, D, P, size):
    P34 = np.hstack([P, np.zeros((3, 1))])
    with open(path, 'w') as f:
        f.write(f'image_width: {size[0]}\nimage_height: {size[1]}\ncamera_name: {name}\n')
        f.write(yaml_matrix('camera_matrix', K))
        f.write(f'distortion_model: {model}\n')
        f.write(yaml_matrix('distortion_coefficients', D.reshape(1, -1)))
        f.write(yaml_matrix('rectification_matrix', np.eye(3)))
        f.write(yaml_matrix('projection_matrix', P34))


def predicted_from_full_frame(model, size, measured, name):
    """
    Report lines comparing a cropped/scaled mode with the full-frame calibration.

    Assumes the mode is the full sensor width scaled by width/1920, with rows cropped
    equally from top and bottom. Agreement within a pixel or two means the full-frame
    calibration can be converted to this mode instead of calibrating it separately.
    """
    if tuple(size) == SENSOR_SIZE or not REPO_OUT:
        return []
    path = os.path.join(REPO_OUT, f'{name}_{SENSOR_SIZE[0]}x{SENSOR_SIZE[1]}_{model}.yaml')
    if not os.path.exists(path):
        return []
    lines = open(path).read().splitlines()
    i = lines.index('camera_matrix:')
    K = np.array([float(v) for v in lines[i + 3].split('[')[1].rstrip(']').split(',')]).reshape(3, 3)
    s = size[0] / SENSOR_SIZE[0]
    off = (SENSOR_SIZE[1] - size[1] / s) / 2  # rows cropped from the top, in sensor pixels
    pred = (K[0, 0] * s, K[1, 1] * s, (K[0, 2] + 0.5) * s - 0.5, (K[1, 2] - off + 0.5) * s - 0.5)
    fx, fy, cx, cy = measured
    return [f'  from full frame   fx fy {pred[0]:.2f} {pred[1]:.2f}, cx cy {pred[2]:.2f} {pred[3]:.2f}   '
            f'(measured - predicted: {fx - pred[0]:+.2f} {fy - pred[1]:+.2f} {cx - pred[2]:+.2f} {cy - pred[3]:+.2f} px; '
            f'assumes a centred crop)']


def calibrate(views, objp, size, args, session):
    """views: list of (name, corners). Writes results; returns {model: (K, D, P)}."""
    names = [v[0] for v in views]
    pts = [v[1] for v in views]
    lines = [f'{len(pts)} views, board {args.cols}x{args.rows} inner corners, square {args.square * 1000:.1f} mm, '
             f'image {size[0]}x{size[1]}']
    print(lines[0])
    results = {}
    for model in ('rational_polynomial', 'equidistant'):
        print(f'{model}: fitting...')
        keep = list(range(len(pts)))
        r = fit(model, objp, pts, size)
        if r is None:
            lines.append(f'\n{model}: failed')
            continue
        # Drop outlier views (misdetections, motion blur), then refit
        errs = np.array([view_error(model, r[1], r[2], objp, p) for p in pts])
        limit = max(3.0 * np.median(errs), 1.0)
        bad = [i for i in keep if errs[i] > limit]
        if bad:
            keep = [i for i in keep if i not in bad]
            r = fit(model, objp, [pts[i] for i in keep], size)
            if r is None:
                lines.append(f'\n{model}: failed after dropping outliers')
                continue
        rms, K, D = r
        kept = [pts[i] for i in keep]
        cv_rms = crossval(model, objp, kept, size)
        P = new_camera_matrix(model, K, D, size)

        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        pix_mm = PIXEL_MM * SENSOR_SIZE[0] / size[0]  # no mode crops horizontally
        hfov = np.degrees(2 * np.arctan(size[0] / 2 / P[0, 0]))
        vfov = np.degrees(2 * np.arctan(size[1] / 2 / P[1, 1]))
        ang, fold, folds = off_axis_angles(model, K, D, size)
        if None in ang.values():
            lens = (f'  lens FOV          INVALID: the model curve turns back at {fold:.1f} deg off-axis, '
                    f'before the image edge or corner; it cannot undistort those pixels. Use another model.')
        else:
            lens = (f'  lens FOV          {ang["left"] + ang["right"]:.1f} H x {ang["top"] + ang["bottom"]:.1f} V, '
                    f'corner {ang["corner"]:.1f} deg off-axis'
                    + (f' (curve turns back at {fold:.1f} deg)' if folds else ''))
        lines += [
            f'\n{model}',
            f'  training RMS      {rms:.3f} px',
            f'  held-out RMS      {cv_rms:.3f} px   (5-fold; compare models on this)',
            f'  views used        {len(keep)}' + (f'  (dropped: {", ".join(names[i] for i in bad)})' if bad else ''),
            f'  fx fy             {fx:.2f} {fy:.2f} px   (focal {fx * pix_mm:.3f} mm at {pix_mm * 1000:.1f} um pixels)',
            f'  cx cy             {cx:.2f} {cy:.2f} px   (centre {size[0] / 2 - 0.5:.1f} {size[1] / 2 - 0.5:.1f})',
            f'  D                 [{", ".join(f"{v:.6g}" for v in D)}]',
            lens,
            *predicted_from_full_frame(model, size, (fx, fy, cx, cy), args.name),
            f'  rectified (P) FOV {hfov:.1f} x {vfov:.1f} deg, f {P[0, 0]:.1f} px',
        ]
        for out in (session, REPO_OUT):
            if not out:
                continue
            path = os.path.join(out, f'{args.name}_{size[0]}x{size[1]}_{model}.yaml')
            if None in ang.values():  # invalid model: no YAML, and remove one from an earlier run
                if os.path.exists(path):
                    os.remove(path)
                continue
            results[model] = (K, D, P)
            os.makedirs(out, exist_ok=True)
            write_camera_info(path, args.name, model, K, D, P, size)
    report = '\n'.join(lines) + '\n'
    print(report)
    for out in (session, REPO_OUT):
        if out:
            with open(os.path.join(out, f'{args.name}_{size[0]}x{size[1]}_report.txt'), 'w') as f:
                f.write(report)
    print(f'written to {session}' + (f' and {REPO_OUT}' if REPO_OUT else ''))
    return results


# ---------------------------------------------------------------- main loops

def load_views(directory, pattern):
    views, size = [], None
    files = sorted(glob.glob(os.path.join(directory, 'view_*.png')))
    if not files:
        raise SystemExit(f'no view_*.png in {directory}')
    for path in files:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        size = (gray.shape[1], gray.shape[0])
        c = detect_accurate(gray, pattern)
        name = os.path.basename(path)
        if c is None:
            print(f'{name}: board not found, skipped')
        else:
            views.append((name, c))
    return views, size


def draw_bar(img, x, y, label, frac):
    w = 160
    cv2.putText(img, label, (x, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(img, (x + 50, y + 2), (x + 50 + w, y + 16), (90, 90, 90), -1)
    col = (60, 200, 60) if frac >= 1 else (0, 170, 255)
    cv2.rectangle(img, (x + 50, y + 2), (x + 50 + int(w * min(frac, 1)), y + 16), col, -1)


def live(args, objp, pattern, results=None):
    """Capture and calibrate; with results given, only preview them (no capturing)."""
    capture = results is None
    session = None
    if capture:
        session = os.path.join(SESSION_ROOT, time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(session, exist_ok=True)
        print(f'saving views to {session}')
    cam = Camera(args.device, args.width, args.height, args.fps)
    size = (args.width, args.height)
    scale = 2 if args.width >= 1600 else 1  # preview detection runs at half resolution for large frames
    win = 'calibrate_camera'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, int(1280 * args.height / args.width))

    views, params = [], []
    prev, still, last_seq = None, 0, -1
    view_mode, maps = (0, {}) if capture else (1, {})
    results = results or {}
    flash = 0.0
    try:
        while True:
            frame, seq = cam.latest()
            if frame is None or seq == last_seq:
                if cv2.waitKey(5) & 0xFF in (ord('q'), 27):
                    break
                continue
            last_seq = seq
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, (size[0] // scale, size[1] // scale), interpolation=cv2.INTER_AREA)
            c = detect_fast(small, pattern) if capture else None
            corners = None if c is None else c * scale

            if corners is not None and prev is not None and len(prev) == len(corners):
                moved = np.max(np.linalg.norm(corners - prev, axis=1))
                still = still + 1 if moved < STILL_PX * 1.5 else 0
            else:
                still = 0
            prev = corners

            key = cv2.waitKey(1) & 0xFF
            want = False
            status = 'no board' if corners is None else 'hold still' if still < STILL_FRAMES else ''
            if not capture:
                status = 'raw'
            if corners is not None:
                p = pose_params(corners, pattern, size)
                novel = all(np.sum(np.abs(p - q)) > MIN_POSE_DIFF for q in params)
                if still >= STILL_FRAMES and novel:
                    want = True
                elif still >= STILL_FRAMES:
                    status = 'pose already captured: move or tilt the board'
                if key == ord(' '):
                    want = True
            if want:
                acc = detect_accurate(gray, pattern, guess=corners)
                if acc is not None:
                    name = f'view_{len(views):03d}.png'
                    cv2.imwrite(os.path.join(session, name), frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                    views.append((name, acc))
                    params.append(pose_params(acc, pattern, size))
                    flash = time.time()
                    still = 0

            # ---- draw
            if view_mode and results:
                model = list(results)[view_mode - 1]
                if model not in maps:
                    K, D, P = results[model]
                    maps[model] = undistort_maps(model, K, D, P, size)
                show = cv2.remap(frame, *maps[model], cv2.INTER_LINEAR)
                status = f'undistorted: {model}  (u: next)'
            else:
                show = frame.copy()
                for _, vc in views:
                    for pt in vc:
                        cv2.circle(show, (int(pt[0]), int(pt[1])), 4, (255, 160, 0), -1)
                if corners is not None:
                    cv2.drawChessboardCorners(show, pattern, corners.reshape(-1, 1, 2).astype(np.float32), True)
            if time.time() - flash < 0.3:
                cv2.rectangle(show, (0, 0), (size[0] - 1, size[1] - 1), (255, 255, 255), 24)

            hud = show[0:200, 0:520]
            hud[:] = (hud * 0.35).astype(np.uint8)
            cv2.putText(show, f'views {len(views)}   {status}', (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            if params:
                arr = np.array(params)
                span = arr.max(axis=0) - arr.min(axis=0)
                for i, (lab, goal) in enumerate(GOAL_RANGE.items()):
                    draw_bar(show, 12, 52 + 22 * i, lab, span[i] / goal)
            cv2.putText(show, 'space capture  c calibrate  u undistort  q quit', (12, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
            cv2.imshow(win, show)

            if key in (ord('q'), 27):
                break
            if key == ord('c') and capture:
                if len(views) < args.min_views:
                    print(f'need at least {args.min_views} views, have {len(views)}')
                else:
                    results = calibrate(views, objp, size, args, session)
                    maps, view_mode = {}, 1 if results else 0
            if key == ord('u') and results:
                view_mode = (view_mode + 1) % (len(results) + 1)
    finally:
        cam.close()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--height', type=int, default=1200)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column')
    ap.add_argument('--square', type=float, default=0.055, help='square size, metres')
    ap.add_argument('--min-views', type=int, default=20)
    ap.add_argument('--name', default='see3cam_24cug', help='camera_name and output file prefix')
    ap.add_argument('--images', help='calibrate from a saved session directory instead of the camera')
    ap.add_argument('--preview', action='store_true',
                    help='with --images: then show the live camera undistorted (u cycles models)')
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = object_points(pattern, args.square)
    if args.images:
        views, size = load_views(args.images, pattern)
        results = calibrate(views, objp, size, args, args.images)
        if args.preview and results:
            live(args, objp, pattern, results)
    else:
        live(args, objp, pattern)


if __name__ == '__main__':
    main()
