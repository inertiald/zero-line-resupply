#!/usr/bin/env python3
"""
Record3D → NVIDIA FoundationPose bridge (standalone, no ROS 2 required).

Modes
─────
  --view            Live preview of RGB + Depth (no saving)
  --save-dir DIR    Stream frames to DIR/ in FoundationPose-ready layout
  --view --save-dir DIR   Both at once

Saved directory layout (what FoundationPose expects)
────────────────────────────────────────────────────
  DIR/
    rgb/          0000.png, 0001.png, …      (BGR, 8-bit)
    depth/        0000.png, 0001.png, …      (uint16 millimetres)
    mask/         0000.png, 0001.png, …      (mono8, 255 = foreground)
    cam_K.txt     3×3 intrinsic matrix
    poses/        0000.txt, 0001.txt, …      (4×4 camera-to-world)

Usage
─────
  python record3d_foundationpose_bridge.py --view
  python record3d_foundationpose_bridge.py --save-dir ./capture
  python record3d_foundationpose_bridge.py --view --save-dir ./capture
  python record3d_foundationpose_bridge.py --view --device 0

Dependencies (all pip-installable)
──────────────────────────────────
  pip install record3d numpy opencv-python
"""

import argparse
import os
import sys
from pathlib import Path
from threading import Event

import cv2
import numpy as np
from record3d import Record3DStream


# ────────────────────────────────────────────────────────────────────────────
#  Helpers
# ────────────────────────────────────────────────────────────────────────────

def intrinsic_mat_from_coeffs(coeffs) -> np.ndarray:
    """Record3D intrinsic coefficients → 3×3 K matrix."""
    return np.array([
        [coeffs.fx,       0.0, coeffs.tx],
        [      0.0, coeffs.fy, coeffs.ty],
        [      0.0,       0.0,       1.0],
    ])


def pose_to_4x4(p) -> np.ndarray:
    """Record3D camera_pose (qx,qy,qz,qw,tx,ty,tz) → 4×4 homogeneous matrix."""
    qx, qy, qz, qw = p.qx, p.qy, p.qz, p.qw
    tx, ty, tz = p.tx, p.ty, p.tz

    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [tx, ty, tz]
    return T


def ensure_dirs(base: Path):
    """Create the FoundationPose capture directory tree."""
    for sub in ('rgb', 'depth', 'mask', 'poses'):
        (base / sub).mkdir(parents=True, exist_ok=True)


# ────────────────────────────────────────────────────────────────────────────
#  Main app
# ────────────────────────────────────────────────────────────────────────────

class Record3DFoundationPoseBridge:
    DEVICE_TYPE_TRUEDEPTH = 0
    DEVICE_TYPE_LIDAR = 1

    def __init__(self, dev_idx: int = 0, view: bool = False,
                 save_dir: str | None = None):
        self.dev_idx = dev_idx
        self.view = view
        self.save_dir = Path(save_dir) if save_dir else None

        self.event = Event()
        self.session: Record3DStream | None = None
        self.frame_idx = 0
        self.intrinsics_saved = False

        if self.save_dir:
            ensure_dirs(self.save_dir)

    # ── Record3D callbacks ──────────────────────────────────────────────
    def _on_new_frame(self):
        self.event.set()

    def _on_stream_stopped(self):
        print('[record3d] Stream stopped.')

    # ── Connection ──────────────────────────────────────────────────────
    def connect(self):
        print('[record3d] Searching for devices …')
        devs = Record3DStream.get_connected_devices()
        print(f'[record3d] {len(devs)} device(s) found')
        for d in devs:
            print(f'  ID: {d.product_id}  UDID: {d.udid}')

        if len(devs) <= self.dev_idx:
            raise RuntimeError(
                f'Cannot connect to device #{self.dev_idx} – '
                f'only {len(devs)} available.'
            )

        self.session = Record3DStream()
        self.session.on_new_frame = self._on_new_frame
        self.session.on_stream_stopped = self._on_stream_stopped
        self.session.connect(devs[self.dev_idx])
        print('[record3d] Connected – streaming.')

    # ── Per-frame processing ────────────────────────────────────────────
    def _process_frame(self):
        depth = self.session.get_depth_frame()             # float32 metres
        rgb   = self.session.get_rgb_frame()               # uint8 H×W×3 RGB
        intrinsics = intrinsic_mat_from_coeffs(
            self.session.get_intrinsic_mat()
        )
        camera_pose = self.session.get_camera_pose()

        # Mirror TrueDepth so it matches LiDAR convention
        if self.session.get_device_type() == self.DEVICE_TYPE_TRUEDEPTH:
            depth = cv2.flip(depth, 1)
            rgb   = cv2.flip(rgb, 1)

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h, w = rgb.shape[:2]

        # ── Save ────────────────────────────────────────────────────────
        if self.save_dir:
            tag = f'{self.frame_idx:04d}'

            # cam_K.txt – written once, reused by FoundationPose
            if not self.intrinsics_saved:
                np.savetxt(
                    str(self.save_dir / 'cam_K.txt'),
                    intrinsics,
                    fmt='%.6f',
                    header='3x3 intrinsic matrix (fx 0 cx / 0 fy cy / 0 0 1)',
                )
                self.intrinsics_saved = True
                print(f'[save] Camera K matrix → {self.save_dir / "cam_K.txt"}')
                print(f'       K =\n{intrinsics}')

            # RGB (BGR 8-bit PNG)
            cv2.imwrite(str(self.save_dir / 'rgb' / f'{tag}.png'), bgr)

            # Depth → uint16 millimetres (FoundationPose convention)
            depth_mm = (depth * 1000.0).astype(np.uint16)
            cv2.imwrite(str(self.save_dir / 'depth' / f'{tag}.png'), depth_mm)

            # Mask – full foreground placeholder
            mask = np.full((h, w), 255, dtype=np.uint8)
            cv2.imwrite(str(self.save_dir / 'mask' / f'{tag}.png'), mask)

            # 4×4 camera-to-world pose
            T = pose_to_4x4(camera_pose)
            np.savetxt(
                str(self.save_dir / 'poses' / f'{tag}.txt'),
                T, fmt='%.8f',
            )

            self.frame_idx += 1
            if self.frame_idx % 30 == 0:
                print(f'[save] {self.frame_idx} frames saved …')

        # ── View ────────────────────────────────────────────────────────
        if self.view:
            # Clip to a tight range so nearby detail isn't washed out.
            # Record3D LiDAR/TrueDepth typically works within ~0.1–5 m.
            d_min = np.percentile(depth[depth > 0], 1)   # ignore zeros (invalid)
            d_max = np.percentile(depth[depth > 0], 99)  # ignore far outliers

            depth_clipped = np.clip(depth, d_min, d_max)
            depth_norm = ((depth_clipped - d_min) / (d_max - d_min) * 255.0)
            depth_vis = depth_norm.astype(np.uint8)

            depth_colour = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)

            # Mark invalid (zero) pixels as black
            depth_colour[depth <= 0] = 0

            cv2.imshow('iPhone RGB', bgr)
            cv2.imshow('iPhone Depth', depth_colour)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                raise KeyboardInterrupt

    # ── Main loop ───────────────────────────────────────────────────────
    def run(self):
        try:
            while True:
                self.event.wait()
                self.event.clear()
                self._process_frame()
        except KeyboardInterrupt:
            print('\n[record3d] Stopped by user.')
        finally:
            if self.view:
                cv2.destroyAllWindows()
            if self.save_dir:
                print(f'[save] {self.frame_idx} total frames saved to {self.save_dir}/')


# ────────────────────────────────────────────────────────────────────────────
#  CLI
# ────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Record3D → NVIDIA FoundationPose bridge',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''\
examples:
  %(prog)s --view                         live preview only
  %(prog)s --save-dir ./capture           save frames for offline FoundationPose
  %(prog)s --view --save-dir ./capture    both at once
''',
    )
    parser.add_argument(
        '--view', action='store_true',
        help='Show live RGB + depth windows (press q to quit)',
    )
    parser.add_argument(
        '--save-dir', type=str, default=None,
        help='Directory to save FoundationPose-ready frames',
    )
    parser.add_argument(
        '--device', type=int, default=0,
        help='Record3D device index (default: 0)',
    )
    args = parser.parse_args()

    if not args.view and args.save_dir is None:
        parser.print_help()
        print('\nError: supply at least --view or --save-dir (or both).')
        sys.exit(1)

    bridge = Record3DFoundationPoseBridge(
        dev_idx=args.device,
        view=args.view,
        save_dir=args.save_dir,
    )
    bridge.connect()
    bridge.run()


if __name__ == '__main__':
    main()