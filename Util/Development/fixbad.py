#fixes bad .ply splat files if they wont import to gaussian 

from plyfile import PlyData, PlyElement
import numpy as np

ply = PlyData.read("input.ply")
v = ply["vertex"].data

names = (["x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2"]
         + [f"f_rest_{i}" for i in range(45)]
         + ["opacity", "scale_0", "scale_1", "scale_2",
            "rot_0", "rot_1", "rot_2", "rot_3"])

out = np.zeros(v.shape, dtype=[(n, "f4") for n in names])
for n in names:
    if n in v.dtype.names:
        out[n] = v[n]   # nx, ny, nz stay 0

PlyData([PlyElement.describe(out, "vertex")], text=False).write("fixed.ply")