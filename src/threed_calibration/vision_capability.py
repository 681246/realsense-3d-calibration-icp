"""
3D vision calibration capability.

This module compares a current point cloud against a master point cloud and
returns the ICP transform, rotation/translation values, and quality metrics.
"""

from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .point_cloud_3d import PointCloudUtils


class ThreeDVisionCapability:
    ICP_DISTANCE_THRESHOLD = 0.02
    ICP_MAX_ITERATION = 200
    ICP_MIN_FITNESS = 0.01
    ICP_IDENTITY_ATOL = 1e-9

    def __init__(self, current_pcd_path=None, current_image_path=None):
        runtime_dir = Path("runtime")
        runtime_dir.mkdir(parents=True, exist_ok=True)
        self._current_pcd_path = Path(current_pcd_path or runtime_dir / "current_scan.pcd")
        self._current_image_path = Path(current_image_path or runtime_dir / "current_image.png")

    def set_path(self, image_path, pcd_path):
        self._current_image_path = Path(image_path)
        self._current_pcd_path = Path(pcd_path)

    def save_snapshot_artifacts(self, snapshot, image_path=None, pcd_path=None):
        target_image_path = Path(image_path or self._current_image_path)
        target_pcd_path = Path(pcd_path or self._current_pcd_path)
        target_image_path.parent.mkdir(parents=True, exist_ok=True)
        target_pcd_path.parent.mkdir(parents=True, exist_ok=True)

        saved = {
            "image_path": "",
            "pcd_path": "",
            "image_saved": False,
            "pcd_saved": False,
            "image_error": None,
            "pcd_error": None,
        }

        image = snapshot.get("image") if isinstance(snapshot, dict) else None
        if image is not None:
            try:
                if not cv2.imwrite(str(target_image_path), image):
                    raise RuntimeError(f"Failed to write image artifact: {target_image_path}")
                saved["image_path"] = str(target_image_path)
                saved["image_saved"] = True
            except Exception as exc:
                saved["image_error"] = str(exc)

        pcd = snapshot.get("pcd") if isinstance(snapshot, dict) else None
        if pcd is not None:
            try:
                if len(pcd.points) == 0:
                    raise RuntimeError("No point cloud points found")
                if not o3d.io.write_point_cloud(str(target_pcd_path), pcd):
                    raise RuntimeError(f"Failed to write point cloud: {target_pcd_path}")
                saved["pcd_path"] = str(target_pcd_path)
                saved["pcd_saved"] = True
            except Exception as exc:
                saved["pcd_error"] = str(exc)

        return saved

    @staticmethod
    def _resolve_existing_pcd_path(path_value, label):
        path = Path(str(path_value or "").strip())
        if not str(path):
            raise RuntimeError(f"{label} PCD path is empty")
        if not path.is_file():
            raise RuntimeError(f"{label} PCD file missing: {path}")
        return path

    @staticmethod
    def _load_non_empty_point_cloud(path, label):
        pcd = o3d.io.read_point_cloud(str(path))
        point_count = len(pcd.points) if pcd is not None else 0
        if point_count <= 0:
            raise RuntimeError(f"{label} PCD has no points: {path}")
        return pcd, point_count

    @classmethod
    def _is_identity_transform(cls, transformation):
        return bool(np.allclose(transformation, np.identity(4), atol=cls.ICP_IDENTITY_ATOL))

    @classmethod
    def _registration_result_quality_error(cls, reg, identity_transform):
        fitness = float(getattr(reg, "fitness", 0.0) or 0.0)
        rmse = float(getattr(reg, "inlier_rmse", 0.0) or 0.0)

        if fitness < cls.ICP_MIN_FITNESS:
            return f"ICP quality too low: fitness={fitness:.6f}, required>={cls.ICP_MIN_FITNESS:.6f}"
        if rmse > cls.ICP_DISTANCE_THRESHOLD:
            return f"ICP quality too low: inlier_rmse={rmse:.6f}, max={cls.ICP_DISTANCE_THRESHOLD:.6f}"
        if identity_transform and fitness < cls.ICP_MIN_FITNESS:
            return f"ICP returned identity transform without acceptable quality: fitness={fitness:.6f}, rmse={rmse:.6f}"

        return None

    def compute_transform(self, master_pcd, current_pcd=None):
        current_pcd = current_pcd or self._current_pcd_path

        master_path = self._resolve_existing_pcd_path(master_pcd, "Master")
        current_path = self._resolve_existing_pcd_path(current_pcd, "Current")

        source_pcd, source_points = self._load_non_empty_point_cloud(current_path, "Current")
        target_pcd, target_points = self._load_non_empty_point_cloud(master_path, "Master")

        transformation, reg = PointCloudUtils.get_transformation_matrix(
            source_pcd,
            target_pcd,
            threshold=self.ICP_DISTANCE_THRESHOLD,
            max_iteration=self.ICP_MAX_ITERATION,
        )

        transformation = np.asarray(transformation)
        identity_transform = self._is_identity_transform(transformation)
        quality_error = self._registration_result_quality_error(reg, identity_transform)
        if quality_error:
            raise RuntimeError(quality_error)

        angles = PointCloudUtils.get_rotation_angles(transformation)

        return {
            "transformation": transformation,
            "rotation_angles": angles,
            "fitness": float(reg.fitness),
            "inlier_rmse": float(reg.inlier_rmse),
            "identity_transform": identity_transform,
            "source_point_count": source_points,
            "target_point_count": target_points,
            "current_pcd_path": str(current_path),
            "master_pcd_path": str(master_path),
        }
