"""Convert the course STEP (Onshape export) into the surface-voxel file VoxelWorld loads. Laptop tool.

  pip install cascadio trimesh
  python tools/sim/step_to_voxels.py "FINAL Course v3.step" tools/sim/final_course_v3.npz

Floor and flat floor decals (< 1 cm tall: pad rings, guideline, launch mark) are not voxelised: the
simulator draws the floor and the blue pad analytically. The pad centre/side come from 'Landing pad 5 ft'.
"""
import sys
import tempfile

import numpy as np

PITCH = 0.025
Z_TOP = 3.1


def main(src, out):
    import cascadio
    import trimesh
    glb = tempfile.mktemp(suffix=".glb")
    cascadio.step_to_glb(src, glb, tol_linear=0.01, tol_angular=0.5)
    sc = trimesh.load(glb)
    lo = np.array([-3.1, -3.1, -0.05])
    shape = np.ceil((np.array([3.1, 3.1, Z_TOP]) - lo) / PITCH).astype(int)
    occ = np.zeros(shape, bool)
    parts, pad = {}, None
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        m = sc.geometry[g].copy()
        m.apply_transform(T)
        b = m.bounds
        parts[node] = b.tolist()
        if node.startswith("Landing pad 5"):
            c = b.mean(axis=0)
            pad = (float(c[0]), float(c[1]), float(min(b[1, 0] - b[0, 0], b[1, 1] - b[0, 1])))
        if node.startswith("Floor") or b[1, 2] - b[0, 2] < 0.01:
            continue
        tri = m.triangles                                   # (N,3,3)
        e = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
        k = np.maximum(1, np.ceil(e / (0.4 * PITCH))).astype(int)
        pts = [tri.mean(axis=1), tri.reshape(-1, 3)]
        for kk in np.unique(k[k > 1]):
            sel = tri[k == kk]
            ii, jj = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1), indexing="ij")
            msk = ii + jj <= kk
            a, bb = ii[msk] / kk, jj[msk] / kk
            w = np.stack([1 - a - bb, a, bb], 1)            # (S,3)
            pts.append(np.einsum("sk,nkd->nsd", w, sel).reshape(-1, 3))
        p = np.concatenate(pts)
        idx = np.floor((p - lo) / PITCH).astype(int)
        ok = np.all((idx >= 0) & (idx < shape), axis=1)
        idx = idx[ok]
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        print("%-32s tris=%6d voxels=%d" % (node[:32], len(tri), len(np.unique(idx, axis=0))))
    np.savez_compressed(out, occ=np.packbits(occ.ravel()), shape=shape, lo=lo, pitch=PITCH,
                        pad=np.array(pad), parts=np.array(parts, dtype=object))
    print("pad", pad, "grid", shape, "occupied", int(occ.sum()))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
