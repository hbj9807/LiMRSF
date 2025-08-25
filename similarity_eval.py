#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Hanbeom Chang, M.S.
Research Assistant of MEIC Laboratory
Department of Mechanical Engineering
The State University of New York, Korea (SUNY Korea)
Email: hanbeom.chang@stonybrook.edu
Website: https://www.meic-lab.com

similarity_eval.py
View-independent 3D similarity evaluation between the pre-simplified highlighted mesh and the simplified mesh,
plus highlight-label preservation and mesh-to-PCD residuals.

Inputs per scene:
  - PCD/<name>.pcd
  - Highlighted Mesh/highlighted_mesh_<name>_relative.ply   (reference mesh, pre-simplified, highlighted)
  - Indices/mesh_blind_spot_indices_<name>_relative.npy     (highlighted vertex indices on the reference mesh)
  - Simplified mesh path is auto-discovered under common folders or can be overridden via CLI.

Outputs:
  - out_dir/<scene>/similarity_metrics.csv
  - out_dir/combined_similarity_summary.csv

python .\similarity_eval.py `
  --out_dir .\out_similarity_vertices `
  --eval_mode vertices `
  --unsigned_normals

python .\similarity_eval.py `
  --out_dir .\out_similarity_sample `
  --eval_mode sample `
  --n_samples_mesh 50000 `
  --n_samples_pcd 30000
"""

import os
import sys
import json
import glob
import argparse
from typing import Dict, List, Tuple, Optional

import numpy as np
from tqdm import tqdm

try:
    import open3d as o3d
except ImportError as e:
    print("ERROR: open3d is required. Install with `pip install open3d`.", file=sys.stderr)
    raise

# -----------------------------
# Manifest (default as provided)
# -----------------------------
DEFAULT_MANIFEST = [
    {
        "name": "adamslab",
        "pcd": "PCD/adamslab1.pcd",
        "relative": {
            "mesh": "Highlighted Mesh/highlighted_mesh_adamslab_relative.ply",
            "idx":  "Indices/mesh_blind_spot_indices_adamslab_relative.npy",
        },
    },
    {
        "name": "ddm",
        "pcd": "PCD/ddm1.pcd",
        "relative": {
            "mesh": "Highlighted Mesh/highlighted_mesh_ddm_relative.ply",
            "idx":  "Indices/mesh_blind_spot_indices_ddm_relative.npy",
        },
    },
    {
        "name": "meiclab",
        "pcd": "PCD/meiclab1.pcd",
        "relative": {
            "mesh": "Highlighted Mesh/highlighted_mesh_meiclab1_relative.ply",
            "idx":  "Indices/mesh_blind_spot_indices_meiclab1_relative.npy",
        },
    },
]

# -----------------------------
# Utilities
# -----------------------------
def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def load_mesh(path: str) -> o3d.geometry.TriangleMesh:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Mesh not found: {path}")
    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) == 0:
        raise ValueError(f"Mesh has no triangles: {path}")
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    return mesh

def load_pcd(path: str) -> o3d.geometry.PointCloud:
    if not os.path.exists(path):
        raise FileNotFoundError(f"PCD not found: {path}")
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise ValueError(f"Point cloud is empty: {path}")
    return pcd

def triangle_areas(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    V = np.asarray(mesh.vertices)
    F = np.asarray(mesh.triangles)
    a = V[F[:,1]] - V[F[:,0]]
    b = V[F[:,2]] - V[F[:,0]]
    areas = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    return areas  # (T,)

def vertex_area_weights(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    """Distribute each triangle's area equally to its 3 vertices (for area-weighted vertex measures)."""
    n = np.asarray(mesh.vertices).shape[0]
    F = np.asarray(mesh.triangles)
    A = triangle_areas(mesh)
    w = np.zeros(n, dtype=np.float64)
    for tri, area in zip(F, A):
        w[tri] += area / 3.0
    return w  # (N,)

def sample_surface_points(mesh: o3d.geometry.TriangleMesh, n: int, seed: int = 42) -> o3d.geometry.PointCloud:
    # Uniform area-weighted sampling
    # (Open3D doesn't accept a seed directly for uniform sampling.)
    pc = mesh.sample_points_uniformly(number_of_points=int(n))
    if not pc.has_normals():
        mesh.compute_vertex_normals()
        pc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30)
        )
    return pc

def build_kdtree_pointcloud(pc: o3d.geometry.PointCloud) -> o3d.geometry.KDTreeFlann:
    return o3d.geometry.KDTreeFlann(pc)

def nn_distances(query_pts: np.ndarray, kd: o3d.geometry.KDTreeFlann, ref_pc: o3d.geometry.PointCloud, desc: str) -> np.ndarray:
    """Nearest neighbor distances from query_pts (Nx3) to ref_pc via KDTreeFlann."""
    ref = np.asarray(ref_pc.points)
    out = np.zeros(len(query_pts), dtype=np.float64)
    for i in tqdm(range(len(query_pts)), desc=f"NN({desc})", leave=False):
        q = query_pts[i]
        _, idx, _ = kd.search_knn_vector_3d(q, 1)
        j = idx[0]
        d = np.linalg.norm(q - ref[j])
        out[i] = d
    return out

def normalize_rows(x: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = np.linalg.norm(x, axis=1, keepdims=True) + eps
    return x / n

def degrees_from_cosine(c: np.ndarray) -> np.ndarray:
    c = np.clip(c, -1.0, 1.0)
    return np.degrees(np.arccos(c))

def mesh_vertices_np(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    return np.asarray(mesh.vertices)

def build_vertex_kd(mesh: o3d.geometry.TriangleMesh) -> Tuple[o3d.geometry.KDTreeFlann, o3d.geometry.PointCloud]:
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(mesh.vertices)))
    kd = o3d.geometry.KDTreeFlann(pc)
    return kd, pc

def sanitize_indices(idx: np.ndarray, n_verts: int):
    """Handle 1-based indices and drop OOB. Return (indices, dropped_count)."""
    idx = np.asarray(idx).astype(np.int64).ravel()
    # Detect 1-based: if max == n_verts and 0 not in idx
    if idx.size > 0 and idx.max() == n_verts and (idx.min() >= 1):
        idx = idx - 1
    # Drop invalid
    keep = (idx >= 0) & (idx < n_verts)
    dropped = int(np.count_nonzero(~keep))
    idx = idx[keep]
    return np.unique(idx), dropped

def boundary_vertices(mask: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """A vertex is boundary if it is in mask and has at least one neighbor vertex outside mask."""
    n = mask.shape[0]
    neighbors = [[] for _ in range(n)]
    for a,b,c in triangles:
        neighbors[a].extend([b,c])
        neighbors[b].extend([a,c])
        neighbors[c].extend([a,b])
    is_boundary = np.zeros(n, dtype=bool)
    for v in np.where(mask)[0]:
        nbrs = neighbors[v]
        if any(not mask[u] for u in nbrs):
            is_boundary[v] = True
    return is_boundary

def summarize_distances(dist: np.ndarray) -> Tuple[float, float]:
    return float(np.mean(dist)), float(np.percentile(dist, 95.0))

def write_csv(path: str, header: List[str], rows: List[List]):
    ensure_dir(os.path.dirname(path))
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")

# -----------------------------
# Point source helper
# -----------------------------
def points_from_mesh(mesh: o3d.geometry.TriangleMesh,
                     mode: str,
                     n_samples: int,
                     seed: int,
                     role: str):
    """
    role: 'query' or 'ref' (for progress labels)
    return: (pc: o3d.geometry.PointCloud, pts: (N,3) ndarray, src: 'vertices'|'samples')
    """
    if mode == "vertices":
        pts = np.asarray(mesh.vertices)
        pc  = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        return pc, pts, "vertices"
    else:
        pc = sample_surface_points(mesh, n_samples, seed=seed)
        pts = np.asarray(pc.points)
        return pc, pts, "samples"

# -----------------------------
# Similarity metrics
# -----------------------------
def chamfer_and_normal(
    mh: o3d.geometry.TriangleMesh,
    ms: o3d.geometry.TriangleMesh,
    n_samples: int = 50000,
    seed: int = 42,
    eval_mode: str = "vertices",
    unsigned_normals: bool = False,
) -> Dict[str, float]:
    """
    Compute bidirectional distances (Chamfer components) and normal consistency.
    eval_mode: 'sample' (surface sampling) or 'vertices' (use all mesh vertices)
    unsigned_normals: if True, use |cos(theta)| (ignore normal sign) to reduce flips' impact
    """
    # Prepare query/reference point sets for both directions
    pc_h, pts_h, src_h = points_from_mesh(mh, eval_mode, n_samples, seed,   role="query(h)")
    pc_s, pts_s, src_s = points_from_mesh(ms, eval_mode, n_samples, seed+1, role="query(s)")

    kd_s = build_kdtree_pointcloud(pc_s)
    kd_h = build_kdtree_pointcloud(pc_h)

    # Distances
    d_h2s = nn_distances(pts_h, kd_s, pc_s, f"h→s[{src_s}]")
    d_s2h = nn_distances(pts_s, kd_h, pc_h, f"s→h[{src_h}]")

    chamfer_l2 = float(np.mean(d_h2s**2) + np.mean(d_s2h**2))
    mean_h2s, p95_h2s = summarize_distances(d_h2s)
    mean_s2h, p95_s2h = summarize_distances(d_s2h)

    # Normal consistency (nearest vertex normals)
    mh.compute_vertex_normals()
    ms.compute_vertex_normals()
    nv_h = normalize_rows(np.asarray(mh.vertex_normals))
    nv_s = normalize_rows(np.asarray(ms.vertex_normals))
    kd_v_h, _ = build_vertex_kd(mh)
    kd_v_s, _ = build_vertex_kd(ms)

    # h-side
    n_h = np.zeros((len(pts_h), 3), dtype=np.float64)
    n_s_near_h = np.zeros_like(n_h)
    for i in tqdm(range(len(pts_h)), desc=f"Normals(h→s) [{src_h}]", leave=False):
        _, idxh, _ = kd_v_h.search_knn_vector_3d(pts_h[i], 1); n_h[i] = nv_h[idxh[0]]
        _, idxs, _ = kd_v_s.search_knn_vector_3d(pts_h[i], 1); n_s_near_h[i] = nv_s[idxs[0]]
    cos_h = np.einsum("ij,ij->i", n_h, n_s_near_h)

    # s-side
    n_s = np.zeros((len(pts_s), 3), dtype=np.float64)
    n_h_near_s = np.zeros_like(n_s)
    for i in tqdm(range(len(pts_s)), desc=f"Normals(s→h) [{src_s}]", leave=False):
        _, idxs, _ = kd_v_s.search_knn_vector_3d(pts_s[i], 1); n_s[i] = nv_s[idxs[0]]
        _, idxh, _ = kd_v_h.search_knn_vector_3d(pts_s[i], 1); n_h_near_s[i] = nv_h[idxh[0]]
    cos_s = np.einsum("ij,ij->i", n_s, n_h_near_s)

    cos_all = np.concatenate([cos_h, cos_s])
    if unsigned_normals:
        cos_all = np.abs(cos_all)
    ang = degrees_from_cosine(cos_all)
    normal_deg_mean = float(np.mean(ang))
    normal_deg_p95  = float(np.percentile(ang, 95.0))

    return {
        "chamfer_l2": chamfer_l2,
        "h2s_mean": mean_h2s, "h2s_p95": p95_h2s,
        "s2h_mean": mean_s2h, "s2h_p95": p95_s2h,
        "normal_deg_mean": normal_deg_mean,
        "normal_deg_p95": normal_deg_p95,
        "n_points_h": int(len(pts_h)),
        "n_points_s": int(len(pts_s)),
        "eval_mode": eval_mode,
        "unsigned_normals": int(bool(unsigned_normals)),
    }

def label_preservation(
    mh: o3d.geometry.TriangleMesh,
    ms: o3d.geometry.TriangleMesh,
    idx_path: str
) -> Dict[str, float]:
    # Highlight vertex indices on mh
    n_h = np.asarray(mh.vertices).shape[0]
    idx = np.load(idx_path)
    idx, dropped = sanitize_indices(idx, n_h)
    mask_h = np.zeros(n_h, dtype=bool)
    mask_h[idx] = True

    # Area on mh using per-vertex area weights
    w_h = vertex_area_weights(mh)
    area_h = float(np.sum(w_h[mask_h]))

    # Transfer to ms via nearest-vertex mapping; mark ms vertices if nearest mh vertex is highlighted
    kd_v_h, _ = build_vertex_kd(mh)
    V_s = mesh_vertices_np(ms)
    mapped_mask_s = np.zeros(V_s.shape[0], dtype=bool)
    for i in tqdm(range(V_s.shape[0]), desc="Map highlight mh→ms(vtx)", leave=False):
        _, idx_near, _ = kd_v_h.search_knn_vector_3d(V_s[i], 1)
        mapped_mask_s[i] = mask_h[idx_near[0]]

    # Area on ms (vertex area weights)
    w_s = vertex_area_weights(ms)
    area_s = float(np.sum(w_s[mapped_mask_s]))

    # Boundary drift: distance between boundaries of highlight regions
    bound_h = boundary_vertices(mask_h, np.asarray(mh.triangles))
    bound_s = boundary_vertices(mapped_mask_s, np.asarray(ms.triangles))
    V_h = mesh_vertices_np(mh)

    B_s = V_s[bound_s]
    dists = []
    if B_s.shape[0] > 0 and np.any(bound_h):
        pc_b_s = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(B_s))
        kd_b_s = o3d.geometry.KDTreeFlann(pc_b_s)
        for v in tqdm(V_h[bound_h], desc="Boundary drift (mh→ms)", leave=False):
            _, idx_near, _ = kd_b_s.search_knn_vector_3d(v, 1)
            nn = B_s[idx_near[0]]
            dists.append(np.linalg.norm(v - nn))

    boundary_mean = float(np.mean(dists)) if dists else float("nan")
    boundary_p95  = float(np.percentile(dists, 95.0)) if dists else float("nan")
    area_recall = (area_s / area_h) if area_h > 0 else float("nan")

    return {
        "highlight_vertices": int(mask_h.sum()),
        "highlight_area_h": area_h,
        "highlight_area_s": area_s,
        "highlight_area_recall": area_recall,
        "highlight_boundary_mean": boundary_mean,
        "highlight_boundary_p95": boundary_p95,
        "idx_dropped": int(dropped),
    }

def mesh_to_pcd_residuals(
    mesh: o3d.geometry.TriangleMesh,
    pcd: o3d.geometry.PointCloud,
    n_samples: int = 30000,
    seed: int = 123,
    eval_mode: str = "vertices",
) -> Dict[str, float]:
    if eval_mode == "vertices":
        pts = np.asarray(mesh.vertices)
        pc_surf = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        src = "vertices"
    else:
        pc_surf = sample_surface_points(mesh, n_samples, seed=seed)
        pts = np.asarray(pc_surf.points)
        src = "samples"

    kd_p = build_kdtree_pointcloud(pcd)
    d = nn_distances(pts, kd_p, pcd, f"mesh→PCD[{src}]")
    mean_d, p95_d = summarize_distances(d)
    return {
        "mesh2pcd_mean": mean_d,
        "mesh2pcd_p95": p95_d,
        "mesh2pcd_n": int(len(pts)),
        "mesh2pcd_mode": eval_mode,
    }

# -----------------------------
# Simplified-mesh path discovery
# -----------------------------
CANDIDATE_PATTERNS = [
    os.path.join("Simplified Mesh", "simplified_mesh_{name}.ply"),
    os.path.join("Simplified Mesh", "simplified_mesh_{name}_relative.ply"),
    os.path.join("Simplified Mesh", "{name}_simplified.ply"),
    os.path.join("Simplified Mesh", "*{name}*.ply"),
    os.path.join("Mesh", "simplified_{name}.ply"),
    os.path.join("Decimated Mesh", "*{name}*.ply"),
    os.path.join(".", "*simplified*{name}*.ply"),
]

def find_simplified_mesh(scene_name: str, base_dir: str = ".") -> Optional[str]:
    # Try exact patterns first
    for pat in CANDIDATE_PATTERNS:
        path = os.path.join(base_dir, pat.format(name=scene_name))
        matches = glob.glob(path)
        if len(matches) == 1 and os.path.exists(matches[0]):
            return matches[0]
        if len(matches) > 1:
            # pick the shortest path as a heuristic
            matches.sort(key=lambda p: (len(p), p))
            return matches[0]
    return None

# -----------------------------
# Main pipeline per scene
# -----------------------------
def evaluate_scene(scene: Dict, out_dir: str, args) -> Dict[str, float]:
    name = scene["name"]
    p_pcd = scene["pcd"]
    p_mh  = scene["relative"]["mesh"]
    p_idx = scene["relative"]["idx"]

    # Simplified path: override > auto-discovery
    p_ms = None
    if args.simp_override:
        # Allow mapping like name:path via JSON or format string
        if os.path.isfile(args.simp_override):
            with open(args.simp_override, "r", encoding="utf-8") as f:
                simp_map = json.load(f)
            p_ms = simp_map.get(name, None)
        else:
            pat = args.simp_override
            p_ms = pat.format(name=name)
    if not p_ms:
        p_ms = find_simplified_mesh(name, base_dir=args.base_dir)

    print(f"[Scene] {name}")
    print(f"  - PCD : {p_pcd}")
    print(f"  - M_h : {p_mh}")
    print(f"  - idx : {p_idx}")
    print(f"  - M_s : {p_ms if p_ms else '(NOT FOUND)'}")

    if not p_ms or not os.path.exists(p_ms):
        raise FileNotFoundError(
            f"Simplified mesh not found for scene '{name}'. "
            f"Use --simp_override with a path pattern like "
            f"'Simplified Mesh/simplified_mesh_{{name}}.ply'."
        )

    # Load data
    mh = load_mesh(p_mh)
    ms = load_mesh(p_ms)
    pcd = load_pcd(p_pcd)

    # 1) Mesh↔Mesh geometry similarity
    geom = chamfer_and_normal(
        mh, ms,
        n_samples=args.n_samples_mesh,
        seed=42,
        eval_mode=args.eval_mode,
        unsigned_normals=args.unsigned_normals,
    )

    # 2) Highlight preservation
    hlp = label_preservation(mh, ms, p_idx)

    # 3) Mesh↔PCD residuals
    res_h = mesh_to_pcd_residuals(
        mh, pcd,
        n_samples=args.n_samples_pcd,
        seed=123,
        eval_mode=args.eval_mode
    )
    res_s = mesh_to_pcd_residuals(
        ms, pcd,
        n_samples=args.n_samples_pcd,
        seed=124,
        eval_mode=args.eval_mode
    )

    # Merge results
    row = {
        "scene": name,
        # geometry
        **geom,
        # highlight
        **hlp,
        # residuals
        "mh2pcd_mean": res_h["mesh2pcd_mean"],
        "mh2pcd_p95":  res_h["mesh2pcd_p95"],
        "ms2pcd_mean": res_s["mesh2pcd_mean"],
        "ms2pcd_p95":  res_s["mesh2pcd_p95"],
    }

    # Per-scene CSV
    scene_dir = os.path.join(out_dir, name)
    ensure_dir(scene_dir)
    header = list(row.keys())
    write_csv(os.path.join(scene_dir, "similarity_metrics.csv"), header, [list(row.values())])

    return row

# -----------------------------
# CLI
# -----------------------------
def parse_args():
    ap = argparse.ArgumentParser(description="3D similarity evaluation between highlighted and simplified meshes.")
    ap.add_argument("--out_dir", type=str, default="./out_similarity", help="Output directory.")
    ap.add_argument("--manifest", type=str, default=None, help="Path to a JSON manifest (if not using DEFAULT_MANIFEST).")
    ap.add_argument("--base_dir", type=str, default=".", help="Base directory to search.")
    ap.add_argument("--simp_override", type=str, default=None,
                    help=("Simplified-mesh path override. "
                          "Either a JSON {name:path} map, or a format string with {name}, "
                          "e.g., 'Simplified Mesh/simplified_mesh_{name}.ply'"))
    ap.add_argument("--n_samples_mesh", type=int, default=50000,
                    help="Surface samples per mesh for Chamfer/Normals (used only when --eval_mode sample).")
    ap.add_argument("--n_samples_pcd", type=int, default=30000,
                    help="Surface samples for mesh→PCD residuals (used only when --eval_mode sample).")
    ap.add_argument("--eval_mode", type=str, default="vertices",
                    choices=["sample", "vertices"],
                    help="Evaluation point source: 'sample' (area-weighted surface samples) "
                         "or 'vertices' (all mesh vertices; default).")
    ap.add_argument("--unsigned_normals", action="store_true",
                    help="Use unsigned normal comparison (|cos|) to reduce normal-flip sensitivity.")
    return ap.parse_args()

def main():
    args = parse_args()
    ensure_dir(args.out_dir)

    # Load manifest
    if args.manifest:
        with open(args.manifest, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    else:
        manifest = DEFAULT_MANIFEST

    combined_rows = []
    for scene in manifest:
        try:
            row = evaluate_scene(scene, args.out_dir, args)
            combined_rows.append(row)
        except Exception as e:
            print(f"[ERROR] While processing '{scene.get('name','?')}': {e}", file=sys.stderr)

    # Combined CSV
    if combined_rows:
        # align columns
        keys = sorted(set(k for r in combined_rows for k in r.keys()))
        rows = [[r.get(k, "") for k in keys] for r in combined_rows]
        write_csv(os.path.join(args.out_dir, "combined_similarity_summary.csv"), keys, rows)
        print(f"[DONE] Combined summary written to {os.path.join(args.out_dir, 'combined_similarity_summary.csv')}")
        print("Per-scene CSV files are saved under each scene's folder.")
    else:
        print("[DONE] No scenes were successfully evaluated.")

if __name__ == "__main__":
    main()
