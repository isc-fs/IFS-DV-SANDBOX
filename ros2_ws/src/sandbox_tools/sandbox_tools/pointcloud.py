"""Fast PointCloud2 <-> numpy for float32 fields.

    pts = read_fields(msg, ("x", "y", "z", "intensity"))   # (N, 4) float32
    msg = make_cloud(header, {"x": x, "y": y, "z": z, "intensity": i})
"""
from __future__ import annotations

import array

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def field_names(msg: PointCloud2) -> list[str]:
    return [f.name for f in msg.fields]


def read_fields(msg: PointCloud2, names=("x", "y", "z", "intensity")) -> np.ndarray:
    """Return an (N, len(names)) float32 array. All requested fields must be FLOAT32."""
    offsets = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
    missing = [n for n in names if n not in offsets]
    if missing:
        raise KeyError(f"cloud has no float32 field(s) {missing}; has {field_names(msg)}")
    n_points = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)
    out = np.empty((n_points, len(names)), dtype=np.float32)
    for j, name in enumerate(names):
        o = offsets[name]
        out[:, j] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return out


def make_cloud(header, fields: dict[str, np.ndarray]) -> PointCloud2:
    """Build an unorganised PointCloud2 from equal-length float arrays."""
    names = list(fields)
    data = np.column_stack([np.asarray(fields[n], dtype=np.float32) for n in names])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = data.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(names)]
    msg.is_bigendian = False
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    # array.array hits rclpy's fast path; plain bytes are checked byte by byte (~50x slower).
    buf = array.array("B")
    buf.frombytes(np.ascontiguousarray(data).tobytes())
    msg.data = buf
    return msg
