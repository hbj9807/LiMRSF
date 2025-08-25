#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hanbeom Chang, M.S.
Research Assistant of MEIC Laboratory
Department of Mechanical Engineering
The State University of New York, Korea (SUNY Korea)
Email: hanbeom.chang@stonybrook.edu
Website: https://www.meic-lab.com

Blind Spot Evaluation (GT-free, proxy-based) — Multi-dataset Runner with tqdm

What this does
--------------
- Loads your mesh (PLY), your model's predicted blind-spot vertex indices (.npy), and the raw PCD/PLY.
- Builds a GT-free density proxy at mesh vertices (multi-scale) from the PCD:
    score = -density  (higher = worse coverage)
    Proxy(q) = (score_s >= thr_s(q)) ∩ dilate(score_l >= thr_l(q), hops=1)
- Compares your prediction mask vs the proxy with Precision / Recall / F1 / IoU.
- Also reports area-weighted and tolerant (boundary-forgiving) variants.
- Sweeps percentiles q and selects an operating point: max F1 subject to Recall ≥ min_recall.
- Runs all three scenes (adamslab, ddm, meiclab) and writes per-scene + combined CSVs.

Notes
-----
- There is NO absolute GT; this evaluates consistency vs a density-based proxy.
- Very large meshes/PCDs can be slow. Consider --voxel_down to downsample the PCD.
- tqdm progress bars are shown for: scenes, per-vertex density counting, and q-sweep.

Run (PowerShell)
----------------
python .\blindspot_eval_test.py `
>>   --out_dir .\out_probe `
>>   --r_small 0.35 --r_large 0.60 `
>>   --q_list "50,60,70,80,85,90,95" `
>>   --min_recall 0.8 `
>>   --tolerant_hops 0
"""

import os
import sys
import json
import argparse
from dataclasses import dataclass, field
from typing import List, Dict, Any

import numpy as np

try:
    import open3d as o3d
except Exception as e:
    raise RuntimeError("This script requires open3d. Install:  pip install open3d") from e

try:
    from tqdm.auto import tqdm
except Exception as e:
    raise RuntimeError("This script requires tqdm. Install:  pip install tqdm") from e


# --------------------------- Default manifest (your paths) ---------------------------

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


# --------------------------- Utils ---------------------------

def ensure_dir(p: str):
    os.makedirs(p, exist_ok=True)


def load_mesh_preserve_indices(path_mesh: str):
    """
    Load mesh WITHOUT any cleanup, so original vertex indexing is preserved.
    (Do NOT call remove_* methods here; they would reorder/renumber vertices.)
    """
    mesh = o3d.io.read_triangle_mesh(path_mesh)
    if mesh.is_empty():
        raise ValueError(f"Mesh is empty: {path_mesh}")
    V = np.asarray(mesh.vertices, dtype=np.float64)
    F = np.asarray(mesh.triangles, dtype=np.int32)
    return V, F


def load_pcd(path_pcd: str, voxel: float = 0.0):
    """Optional voxel downsampling can greatly speed up density queries."""
    pcd = o3d.io.read_point_cloud(path_pcd)
    if pcd.is_empty():
        raise ValueError(f"Point cloud is empty: {path_pcd}")
    if voxel and voxel > 0.0:
        pcd = pcd.voxel_down_sample(voxel)
    return pcd


def build_vertex_adjacency(n_verts: int, faces: np.ndarray):
    """Adjacency as list[set[int]]; used for tolerant hop-dilation."""
    adj = [set() for _ in range(n_verts)]
    # Faces can be large; use tqdm for visibility
    for tri in tqdm(faces, desc="Building adjacency (faces)", unit="face", leave=False):
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        adj[a].add(b); adj[a].add(c)
        adj[b].add(a); adj[b].add(c)
        adj[c].add(a); adj[c].add(b)
    return adj


def vertex_area_from_faces(verts: np.ndarray, faces: np.ndarray):
    """
    Triangle area is equally distributed to its 3 vertices.
    Vectorized with np.add.at (faster than Python loops).
    """
    f = faces.astype(int)
    p0 = verts[f[:, 0]]
    p1 = verts[f[:, 1]]
    p2 = verts[f[:, 2]]
    tri_areas = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0), axis=1)
    area = np.zeros((len(verts),), dtype=float)
    # scatter-add per-vertex
    np.add.at(area, f[:, 0], tri_areas / 3.0)
    np.add.at(area, f[:, 1], tri_areas / 3.0)
    np.add.at(area, f[:, 2], tri_areas / 3.0)
    return area


def dilate_mask_by_hops(mask: np.ndarray, adjacency, hops: int = 1):
    """Hop-based dilation on mesh graph; used for tolerant (boundary forgiving) metrics."""
    if hops <= 0:
        return mask.copy()
    from collections import deque
    n = len(mask)
    out = mask.copy()
    frontier = deque(np.nonzero(mask)[0].tolist())
    remaining = np.full(n, -1, dtype=int)
    for idx in np.nonzero(mask)[0]:
        remaining[idx] = hops
    # Frontier can be large; a bar per-pop is too noisy. Keep it simple here.
    while frontier:
        u = frontier.popleft()
        h = remaining[u]
        if h <= 0:
            continue
        for v in adjacency[u]:
            if remaining[v] < h - 1:
                remaining[v] = h - 1
                frontier.append(v)
                out[v] = True
    return out


def confusion_counts(gt_mask: np.ndarray, pred_mask: np.ndarray, weights: np.ndarray = None):
    if gt_mask.dtype != bool or pred_mask.dtype != bool:
        raise ValueError("gt_mask and pred_mask must be boolean arrays.")
    if gt_mask.shape != pred_mask.shape:
        raise ValueError("gt_mask and pred_mask must have the same shape.")
    if weights is not None and weights.shape != gt_mask.shape:
        raise ValueError("weights must have the same shape as masks.")
    w = np.ones_like(gt_mask, dtype=float) if weights is None else weights.astype(float)
    tp = float(np.sum(w[(gt_mask) & (pred_mask)]))
    fp = float(np.sum(w[(~gt_mask) & (pred_mask)]))
    fn = float(np.sum(w[(gt_mask) & (~pred_mask)]))
    tn = float(np.sum(w[(~gt_mask) & (~pred_mask)]))
    return tp, fp, fn, tn


def precision_recall_f1_iou(tp, fp, fn):
    prec = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    rec  = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1   = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0
    iou  = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0.0
    return prec, rec, f1, iou


def percentile_threshold(score: np.ndarray, q: float):
    """Return threshold value so that >= thr selects the top q% (q in [0, 100])."""
    return np.quantile(score, float(q) / 100.0)


def per_vertex_density_counts(vertices: np.ndarray, pcd, radius: float):
    """
    Count PCD points within 'radius' for each vertex.
    NOTE: This is O(N) radius queries; can be slow for huge meshes. tqdm shows progress.
    """
    kdt = o3d.geometry.KDTreeFlann(pcd)
    counts = np.zeros((len(vertices),), dtype=np.int32)
    for i, v in enumerate(tqdm(vertices, desc=f"Radius count r={radius:.2f}", unit="vtx", leave=False)):
        c, idx, d2 = kdt.search_radius_vector_3d(v, radius)
        counts[i] = int(c)
    return counts


def load_pred_mask_from_indices(idx_path: str, N: int):
    """
    Robust loader for prediction indices:
    - Detect and shift 1-based indices to 0-based when evident.
    - Drop out-of-range indices with a warning.
    """
    raw = np.load(idx_path)
    idx = np.array(raw).astype(np.int64).ravel()
    shifted = False

    # Heuristic: 1-based if min>=1 and max==N (exactly reaching N)
    if idx.size > 0 and idx.min() >= 1 and idx.max() == N:
        idx = idx - 1
        shifted = True

    invalid = (idx < 0) | (idx >= N)
    drop = int(np.sum(invalid))
    if drop > 0:
        idx = idx[~invalid]

    mask = np.zeros((N,), dtype=bool)
    if idx.size > 0:
        mask[idx] = True

    info = {
        "orig_count": int(raw.size),
        "used_count": int(idx.size),
        "dropped": drop,
        "shifted_from_1_based": shifted,
    }
    return mask, info


# --------------------------- Config & core evaluation ---------------------------

@dataclass
class EvalConfig:
    r_small: float = 0.25                 # small radius (m)
    r_large: float = 0.40                 # large radius (m)
    q_list: List[int] = field(default_factory=lambda: [70, 80, 85, 90, 95])
    tolerant_hops: int = 1               # hop dilation for tolerant metrics
    min_recall: float = 0.80             # operating point constraint
    voxel_down: float = 0.0              # optional PCD voxel downsampling (m)
    save_masks: bool = False             # debug option (not used here)

def make_proxy_masks(V: np.ndarray, F: np.ndarray, pcd, cfg: EvalConfig):
    # Density at two radii (per area)
    counts_s = per_vertex_density_counts(V, pcd, cfg.r_small)
    counts_l = per_vertex_density_counts(V, pcd, cfg.r_large)
    dens_s = counts_s / (np.pi * cfg.r_small * cfg.r_small + 1e-12)
    dens_l = counts_l / (np.pi * cfg.r_large * cfg.r_large + 1e-12)
    score_s = -dens_s
    score_l = -dens_l

    # Adjacency once; used for dilation
    adj = build_vertex_adjacency(len(V), F)

    masks = {}
    for q in tqdm(cfg.q_list, desc="q-sweep (proxy build)", unit="q", leave=False):
        thr_s = percentile_threshold(score_s, q)
        thr_l = percentile_threshold(score_l, q)
        Ms = (score_s >= thr_s)
        Ml = (score_l >= thr_l)
        Ml_d = dilate_mask_by_hops(Ml, adj, hops=1) if cfg.tolerant_hops > 0 else Ml
        masks[q] = Ms & Ml_d

    extras = {"score_s": score_s, "score_l": score_l, "dens_s": dens_s, "dens_l": dens_l}
    return masks, extras


def evaluate_against_proxy(pred_mask: np.ndarray, proxy_mask: np.ndarray,
                           V: np.ndarray, F: np.ndarray, cfg: EvalConfig):
    area_w = vertex_area_from_faces(V, F)
    adj = build_vertex_adjacency(len(V), F)

    # Base (vertex-level) metrics
    tp, fp, fn, tn = confusion_counts(proxy_mask, pred_mask, weights=None)
    p, r, f1, iou = precision_recall_f1_iou(tp, fp, fn)

    # Area-weighted variants
    tp_w, fp_w, fn_w, tn_w = confusion_counts(proxy_mask, pred_mask, weights=area_w)
    p_w, r_w, f1_w, iou_w = precision_recall_f1_iou(tp_w, fp_w, fn_w)

    # Tolerant (dilate proxy)
    proxy_tol = dilate_mask_by_hops(proxy_mask, adj, hops=cfg.tolerant_hops)
    tp_t, fp_t, fn_t, tn_t = confusion_counts(proxy_tol, pred_mask, weights=None)
    p_t, r_t, f1_t, iou_t = precision_recall_f1_iou(tp_t, fp_t, fn_t)

    return {
        "precision": p, "recall": r, "f1": f1, "iou": iou,
        "precision_w": p_w, "recall_w": r_w, "f1_w": f1_w, "iou_w": iou_w,
        "precision_tol": p_t, "recall_tol": r_t, "f1_tol": f1_t, "iou_tol": iou_t,
        "tp": tp, "fp": fp, "fn": fn,
    }


def choose_operating_point(results_by_q: Dict[int, Dict[str, float]], min_recall: float):
    """
    Choose q* that maximizes F1 among results with recall >= min_recall.
    If none satisfy, choose global max F1.
    """
    eligible = [(q, m) for q, m in results_by_q.items() if m["recall"] >= min_recall]
    if len(eligible) == 0:
        q_star, m_star = max(results_by_q.items(), key=lambda kv: kv[1]["f1"])
    else:
        q_star, m_star = max(eligible, key=lambda kv: kv[1]["f1"])
    return q_star, m_star


# --------------------------- Scene runner ---------------------------

def run_scene(scene: Dict[str, Any], cfg: EvalConfig, out_dir: str):
    name = scene["name"]
    pcd_path = scene["pcd"]
    mesh_path = scene["relative"]["mesh"]
    idx_path  = scene["relative"]["idx"]

    print(f"\n[Scene] {name}")
    print(f"  mesh: {mesh_path}")
    print(f"  pcd : {pcd_path}")
    print(f"  pred: {idx_path}")

    # Load data
    V, F = load_mesh_preserve_indices(mesh_path)
    pcd  = load_pcd(pcd_path, voxel=cfg.voxel_down)

    # Prediction mask (robust to 1-based and OOB indices)
    pred_mask, info = load_pred_mask_from_indices(idx_path, N=len(V))
    if info["shifted_from_1_based"]:
        print("  [note] pred indices were 1-based -> shifted to 0-based")
    if info["dropped"] > 0:
        print(f"  [warn] dropped {info['dropped']} invalid indices "
              f"(used {info['used_count']} / original {info['orig_count']})")

    # Build proxy masks (with progress bars)
    proxy_masks, _ = make_proxy_masks(V, F, pcd, cfg)

    # Evaluate for each q (progress bar)
    per_q = {}
    for q in tqdm(sorted(proxy_masks.keys()), desc="q-sweep (evaluation)", unit="q", leave=False):
        proxy = proxy_masks[q]
        per_q[q] = evaluate_against_proxy(pred_mask, proxy, V, F, cfg)

    # Select operating point
    q_star, m_star = choose_operating_point(per_q, cfg.min_recall)

    # Write outputs
    scene_dir = os.path.join(out_dir, name)
    ensure_dir(scene_dir)

    with open(os.path.join(scene_dir, "summary_at_operating_point.csv"), "w", encoding="utf-8") as f:
        f.write("name,q_star,min_recall,precision,recall,f1,iou,precision_w,recall_w,f1_w,iou_w,precision_tol,recall_tol,f1_tol,iou_tol\n")
        f.write(f"{name},{q_star},{cfg.min_recall},"
                f"{m_star['precision']:.6f},{m_star['recall']:.6f},{m_star['f1']:.6f},{m_star['iou']:.6f},"
                f"{m_star['precision_w']:.6f},{m_star['recall_w']:.6f},{m_star['f1_w']:.6f},{m_star['iou_w']:.6f},"
                f"{m_star['precision_tol']:.6f},{m_star['recall_tol']:.6f},{m_star['f1_tol']:.6f},{m_star['iou_tol']:.6f}\n")

    with open(os.path.join(scene_dir, "sweep_q_results.csv"), "w", encoding="utf-8") as f:
        f.write("q,precision,recall,f1,iou,precision_w,recall_w,f1_w,iou_w,precision_tol,recall_tol,f1_tol,iou_tol,tp,fp,fn\n")
        for q in sorted(per_q.keys()):
            m = per_q[q]
            f.write(f"{q},{m['precision']:.6f},{m['recall']:.6f},{m['f1']:.6f},{m['iou']:.6f},"
                    f"{m['precision_w']:.6f},{m['recall_w']:.6f},{m['f1_w']:.6f},{m['iou_w']:.6f},"
                    f"{m['precision_tol']:.6f},{m['recall_tol']:.6f},{m['f1_tol']:.6f},{m['iou_tol']:.6f},"
                    f"{int(m['tp'])},{int(m['fp'])},{int(m['fn'])}\n")

    return {
        "name": name,
        "q_star": int(q_star),
        "precision": float(m_star["precision"]),
        "recall": float(m_star["recall"]),
        "f1": float(m_star["f1"]),
        "iou": float(m_star["iou"]),
        "precision_w": float(m_star["precision_w"]),
        "recall_w": float(m_star["recall_w"]),
        "f1_w": float(m_star["f1_w"]),
        "iou_w": float(m_star["iou_w"]),
        "precision_tol": float(m_star["precision_tol"]),
        "recall_tol": float(m_star["recall_tol"]),
        "f1_tol": float(m_star["f1_tol"]),
        "iou_tol": float(m_star["iou_tol"]),
    }


# --------------------------- CLI & interactive ---------------------------

@dataclass
class ArgsCfg:
    manifest: str = ""
    out_dir: str = ""
    r_small: float = 0.25
    r_large: float = 0.40
    q_list: str = "70,80,85,90,95"
    tolerant_hops: int = 1
    min_recall: float = 0.80
    voxel_down: float = 0.0
    save_masks: bool = False
    interactive: bool = False

def parse_args():
    ap = argparse.ArgumentParser(description="Evaluate blind-spot predictions vs GT-free density proxy on multiple scenes.")
    ap.add_argument("--manifest", type=str, default="", help="Path to JSON manifest. If omitted, uses default scenes.")
    ap.add_argument("--out_dir", type=str, default="", help="Output directory (default: ./out)")
    ap.add_argument("--r_small", type=float, default=0.25, help="Small radius (m)")
    ap.add_argument("--r_large", type=float, default=0.40, help="Large radius (m)")
    ap.add_argument("--q_list", type=str, default="70,80,85,90,95", help="Comma-separated percentiles for score=-density")
    ap.add_argument("--tolerant_hops", type=int, default=1, help="Hop dilation for tolerant metrics")
    ap.add_argument("--min_recall", type=float, default=0.80, help="Minimum recall constraint for operating point")
    ap.add_argument("--voxel_down", type=float, default=0.0, help="Optional voxel downsampling for PCD (m)")
    ap.add_argument("--save_masks", action="store_true", help="(Debug) Save masks/scores (not used here)")
    ap.add_argument("--interactive", action="store_true", help="Prompt for parameters if desired")
    return ap.parse_args()


def interactive_inputs(args):
    print("\n[Interactive] Press Enter to accept defaults.\n")

    def ask(prompt, cast, default):
        s = input(f"{prompt} [{default}]: ").strip()
        if s == "":
            return default
        try:
            return cast(s)
        except Exception:
            print("  -> Invalid; using default.")
            return default

    out_dir = args.out_dir or (input("Output folder [./out]: ").strip() or "./out")
    r_small = ask("r_small (m)", float, args.r_small)
    r_large = ask("r_large (m)", float, args.r_large)
    q_list_in = input(f"q_list (e.g., 70,80,85,90,95) [{args.q_list}]: ").strip() or args.q_list
    q_list = [int(x.strip()) for x in q_list_in.split(",") if x.strip()]
    tolerant_hops = ask("tolerant_hops (int)", int, args.tolerant_hops)
    min_recall = ask("min_recall (0..1)", float, args.min_recall)
    voxel_down = ask("voxel_down (m; optional)", float, args.voxel_down)

    cfg = EvalConfig(
        r_small=r_small, r_large=r_large, q_list=q_list,
        tolerant_hops=tolerant_hops, min_recall=min_recall,
        voxel_down=voxel_down, save_masks=args.save_masks
    )
    return out_dir, cfg


def main():
    args = parse_args()

    # Manifest
    if args.manifest:
        with open(args.manifest, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    else:
        manifest = DEFAULT_MANIFEST

    # Interactive?
    use_interactive = args.interactive or (len(sys.argv) == 1 and not args.out_dir)
    if use_interactive:
        out_dir, cfg = interactive_inputs(args)
    else:
        out_dir = args.out_dir or "./out"
        q_list = [int(x.strip()) for x in args.q_list.split(",") if x.strip()]
        cfg = EvalConfig(
            r_small=args.r_small, r_large=args.r_large, q_list=q_list,
            tolerant_hops=args.tolerant_hops, min_recall=args.min_recall,
            voxel_down=args.voxel_down, save_masks=args.save_masks
        )

    ensure_dir(out_dir)

    # Run scenes with a nice progress bar
    rows = []
    for scene in tqdm(manifest, desc="Scenes", unit="scene"):
        # Quick path existence warnings
        for key, path in [("pcd", scene["pcd"]),
                          ("mesh", scene["relative"]["mesh"]),
                          ("idx", scene["relative"]["idx"])]:
            if not os.path.exists(path):
                print(f"[warn] {scene['name']} missing {key} path: {path}")

        try:
            res = run_scene(scene, cfg, out_dir)
            rows.append(res)
        except Exception as e:
            print(f"[error] Exception while processing {scene['name']}: {e}")

    # Combined CSV
    comb_path = os.path.join(out_dir, "combined_summary.csv")
    with open(comb_path, "w", encoding="utf-8") as f:
        f.write("name,q_star,precision,recall,f1,iou,precision_w,recall_w,f1_w,iou_w,precision_tol,recall_tol,f1_tol,iou_tol\n")
        for r in rows:
            f.write(f"{r['name']},{r['q_star']},{r['precision']:.6f},{r['recall']:.6f},{r['f1']:.6f},{r['iou']:.6f},"
                    f"{r['precision_w']:.6f},{r['recall_w']:.6f},{r['f1_w']:.6f},{r['iou_w']:.6f},"
                    f"{r['precision_tol']:.6f},{r['recall_tol']:.6f},{r['f1_tol']:.6f},{r['iou_tol']:.6f}\n")

    print(f"\n[Done] Combined summary: {comb_path}")
    print("Per-scene outputs: summary_at_operating_point.csv and sweep_q_results.csv in each scene folder.")


if __name__ == "__main__":
    main()
