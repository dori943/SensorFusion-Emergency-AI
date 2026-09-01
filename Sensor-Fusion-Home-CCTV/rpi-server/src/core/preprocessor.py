"""Low-allocation point-cloud preprocessing for the LiDAR stream."""

from __future__ import annotations

from typing import Final

import open3d as o3d


DEFAULT_VOXEL_SIZE: Final[float] = 0.05


def remove_ground_and_downsample(
    cloud: o3d.geometry.PointCloud,
    *,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    distance_threshold: float = 0.04,
    ransac_n: int = 3,
    num_iterations: int = 100,
) -> o3d.geometry.PointCloud:
    """Return a compact cloud with the dominant ground plane removed.

    RANSAC operates on Open3D's native point buffer, so this function never
    materializes the input as a NumPy array.  Only the non-ground cloud and
    the final voxel cloud are allocated; this is important on a Raspberry Pi.
    """
    if voxel_size <= 0:
        raise ValueError("voxel_size must be greater than zero")
    if distance_threshold <= 0:
        raise ValueError("distance_threshold must be greater than zero")
    if ransac_n < 3:
        raise ValueError("ransac_n must be at least 3")

    point_count = len(cloud.points)
    # RANSAC needs at least ransac_n points.  Downsample directly when a plane
    # cannot be estimated, avoiding a needless empty/index allocation.
    if point_count < ransac_n:
        return cloud.voxel_down_sample(voxel_size)

    _, ground_indices = cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )

    # invert=True asks Open3D to keep non-ground points without building a
    # Python list of every point index.
    obstacles = cloud.select_by_index(ground_indices, invert=True)
    # Voxelization is deliberately last: fewer points reach the allocator and
    # renderer, while the ground-plane fit still sees the original detail.
    return obstacles.voxel_down_sample(voxel_size)


def preprocess_point_cloud(
    cloud: o3d.geometry.PointCloud,
    **kwargs: float | int,
) -> o3d.geometry.PointCloud:
    """Compatibility-friendly name for the standard preprocessing pipeline."""
    return remove_ground_and_downsample(cloud, **kwargs)
