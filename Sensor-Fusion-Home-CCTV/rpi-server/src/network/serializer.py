"""Binary framing for point clouds sent to the mobile WebView."""

from __future__ import annotations

import numpy as np


# The count itself is a float32.  Above this value consecutive integers are no
# longer exactly representable, so rejecting it prevents ambiguous frames.
_MAX_EXACT_FLOAT32_INTEGER = 16_777_216


def pack_xyz_float32(xyz: np.ndarray) -> bytes:
    """Pack ``(N, 3)`` xyz values as ``[N, x1, y1, z1, ...]`` little-endian f32.

    A single contiguous output allocation is made.  ``np.ascontiguousarray``
    reuses a correctly laid-out float32 input, and makes one unavoidable
    conversion copy only for strided or non-float32 camera/LiDAR input.
    """
    points = np.asarray(xyz)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")

    count = points.shape[0]
    if count > _MAX_EXACT_FLOAT32_INTEGER:
        raise ValueError("point count cannot be represented exactly as float32")

    # Explicit little-endian framing keeps ARM (Raspberry Pi) and WebView
    # decoding deterministic regardless of the host's native byte order.
    points_f32 = np.ascontiguousarray(points, dtype=np.dtype("<f4"))
    frame = np.empty(1 + points_f32.size, dtype=np.dtype("<f4"))
    frame[0] = count
    frame[1:] = points_f32.reshape(-1)
    return frame.tobytes()


# Short alias for network call sites that use "serialize" terminology.
serialize_xyz = pack_xyz_float32
