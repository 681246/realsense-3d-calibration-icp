"""
Point cloud utility functions for 3D calibration.

The main portfolio value is the ICP registration path:
current point cloud -> master point cloud -> 4x4 transformation -> tx/ty/tz/rx/ry/rz.
"""

from copy import deepcopy

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R


class PointCloudUtils:
    """Open3D point-cloud construction, registration, and transform helpers."""

    @staticmethod
    def zed_to_point_cloud(points_xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_xyz)
        return pcd

    @staticmethod
    def construct_point_cloud(depth_image, color_image, intrinsics, depth_scale=1.0):
        o3d_depth = o3d.geometry.Image(depth_image)
        o3d_color = o3d.geometry.Image(color_image)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color,
            o3d_depth,
            depth_scale=1 / depth_scale,
            convert_rgb_to_intensity=False,
        )

        o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsics["width"],
            height=intrinsics["height"],
            fx=intrinsics["fx"],
            fy=intrinsics["fy"],
            cx=intrinsics["cx"],
            cy=intrinsics["cy"],
        )

        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d_intrinsics)

    @staticmethod
    def save_point_cloud(point_cloud, file_name):
        return o3d.io.write_point_cloud(str(file_name), point_cloud)

    @staticmethod
    def _load_point_cloud(pcd_or_path):
        if isinstance(pcd_or_path, o3d.geometry.PointCloud):
            return pcd_or_path
        if isinstance(pcd_or_path, str):
            return o3d.io.read_point_cloud(pcd_or_path)
        raise TypeError("Point cloud must be open3d.geometry.PointCloud or file path string")

    @classmethod
    def get_transformation_matrix(
        cls,
        source,
        target,
        initial_transform=np.identity(4),
        point_to_plane=False,
        max_iteration=200,
        threshold=0.02,
    ):
        """Compute transform mapping source/current scan onto target/master scan."""
        source_pcd = cls._load_point_cloud(source)
        target_pcd = cls._load_point_cloud(target)

        if point_to_plane:
            source_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            target_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iteration
        )

        reg = o3d.pipelines.registration.registration_icp(
            source_pcd,
            target_pcd,
            threshold,
            initial_transform,
            estimation,
            criteria,
        )

        if reg.transformation is None:
            raise RuntimeError("ICP failed to produce a transformation matrix")

        return deepcopy(reg.transformation), reg

    @staticmethod
    def get_rotation_angles(transformation_matrix):
        """Return translation and Euler rotation values from a 4x4 transform."""
        rot_mat = transformation_matrix[:3, :3]
        rot = R.from_matrix(rot_mat)
        rx, ry, rz = rot.as_euler("xyz", degrees=True)

        return {
            "tx": float(transformation_matrix[0, 3]),
            "ty": float(transformation_matrix[1, 3]),
            "tz": float(transformation_matrix[2, 3]),
            "rx": float(rx),
            "ry": float(ry),
            "rz": float(rz),
        }
