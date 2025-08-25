#!/usr/bin/env python3
"""
LiMRSF.py

by Hanbeom Chang, M.S.
Research Assistant of MEIC Laboratory
Department of Mechanical Engineering
The State University of New York, Korea (SUNY Korea)
Email: hanbeom.chang@stonybrook.edu
Website: https://www.meic-lab.com

End-to-end LiMRSF mesh pipeline in ONE node:

1) Outlier removal (statistical)
2) Normal estimation (no cropping — removed per manuscript)
3) Poisson reconstruction
4) RGB transfer to mesh vertices via k-NN (RGB-textured)
5) Low-density (blind-spot) mask on vertices
6) Small-component filtering to reduce false positives
7) Save highlighted mesh as PLY   <-- stop point for pre-simplified artifact
8) Convert mesh to ROS message (helper)
9) QEM mesh simplification (ablation-ready target_tris)
10) Save simplified mesh as PLY
11) Publish to Unity via ROS-TCP (/mesh_data, /mesh_ply_file)

"""

import os
import time
import numpy as np
import open3d as o3d
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from mesh_publisher.msg import CustomMesh
from collections import defaultdict, deque


# ----------------------------
# Core processing primitives
# ----------------------------

def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """Statistical outlier removal to stabilise normals/Poisson inputs."""
    rospy.loginfo(f"[limrsf] Outlier removal (k={nb_neighbors}, std={std_ratio})")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd.select_by_index(ind)

def compute_normals(pcd, radius=0.5, max_nn=30):
    """Estimate oriented normals for Poisson reconstruction."""
    rospy.loginfo(f"[limrsf] Normal estimation (radius={radius}, max_nn={max_nn})")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    pcd.orient_normals_consistent_tangent_plane(100)
    return pcd

def poisson_reconstruction(pcd, depth=13):
    """Poisson surface reconstruction → (mesh, per-vertex densities)."""
    rospy.loginfo(f"[limrsf] Poisson reconstruction (depth={depth})")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    return mesh, np.asarray(densities)

def kdtree_rgb_transfer(pcd, mesh, k=3, saturation_boost=1.0):
    """Transfer RGB from point cloud to mesh vertices via k-NN (RGB-textured mesh)."""
    if len(pcd.colors) == 0:
        raise ValueError("Point cloud has no RGB colours to transfer.")
    rospy.loginfo(f"[limrsf] RGB transfer (k={k}, sat_boost={saturation_boost})")

    pcolors = np.asarray(pcd.colors)
    verts = np.asarray(mesh.vertices)
    vcols = np.zeros((len(verts), 3), dtype=np.float32)
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    for i, v in enumerate(verts):
        _, idx, _ = kdtree.search_knn_vector_3d(v, k)
        vcols[i] = pcolors[idx].mean(axis=0)

    if saturation_boost != 1.0:
        vcols = np.clip(vcols * saturation_boost, 0.0, 1.0)
    mesh.vertex_colors = o3d.utility.Vector3dVector(vcols)
    return mesh

def compute_low_density_mask(densities, mode="relative", tau=0.7, percentile=40.0):
    """
    Boolean mask for blind-spot candidate vertices using density threshold.

    mode == "relative"   → thr = mean(densities) * tau
    mode == "percentile" → thr = percentile(densities, p)
    """
    if mode == "percentile":
        thr = np.percentile(densities, percentile)
        rospy.loginfo(f"[limrsf] Threshold = percentile({percentile:.1f}) → {thr:.6f}")
    else:
        thr = densities.mean() * tau
        rospy.loginfo(f"[limrsf] Threshold = mean * tau (tau={tau:.3f}) → {thr:.6f}")
    return densities < thr

def filter_small_components(mask, mesh, min_size=50):
    """
    Reduce false positives by removing small connected components
    in the low-density set (vertex adjacency graph).
    """
    if min_size <= 1:
        return mask

    tris = np.asarray(mesh.triangles)
    nverts = len(mesh.vertices)
    adj = [[] for _ in range(nverts)]
    for a, b, c in tris:
        adj[a].extend([b, c])
        adj[b].extend([a, c])
        adj[c].extend([a, b])

    visited = np.zeros(nverts, dtype=bool)
    new_mask = mask.copy()

    for v in np.where(mask)[0]:
        if visited[v]:
            continue
        stack = [v]
        comp = []
        visited[v] = True
        while stack:
            u = stack.pop()
            if not mask[u]:
                continue
            comp.append(u)
            for w in adj[u]:
                if not visited[w]:
                    visited[w] = True
                    stack.append(w)
        if len(comp) < min_size:
            new_mask[comp] = False

    rospy.loginfo(f"[limrsf] Small-component filter removed "
                  f"{int(mask.sum() - new_mask.sum())} vertices (<{min_size})")
    return new_mask

def paint_mask_on_mesh(mesh, mask, colour_low=(1.0, 0.0, 0.0)):
    """Colour low-density vertices for MR highlighting (default red)."""
    cols = np.asarray(mesh.vertex_colors)
    if cols.shape != (len(mesh.vertices), 3):
        cols = np.ones((len(mesh.vertices), 3), dtype=np.float32)
    cols[mask] = np.array(colour_low, dtype=np.float32)
    mesh.vertex_colors = o3d.utility.Vector3dVector(cols)
    return mesh

def mesh_to_ros_msg(mesh):
    """Convert Open3D mesh to mesh_publisher/CustomMesh."""
    msg = CustomMesh()
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    msg.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in verts]
    msg.triangles = tris.flatten().astype(np.uint32).tolist()
    if len(mesh.vertex_colors) > 0:
        cols = np.asarray(mesh.vertex_colors)
        msg.vertex_colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in cols]
    return msg

def simplify_mesh_qem(mesh, target_tris=10000):
    """Quadric error metric decimation for MR performance; ablation-ready."""
    rospy.loginfo(f"[limrsf] QEM simplify → target={target_tris} tris")
    t = time.perf_counter()
    simp = mesh.simplify_quadric_decimation(target_number_of_triangles=int(target_tris))
    rospy.loginfo(f"[limrsf] QEM done: {len(simp.triangles)} tris in {time.perf_counter()-t:.3f}s")
    return simp

def save_ply(mesh, path):
    o3d.io.write_triangle_mesh(path, mesh)
    rospy.loginfo(f"[limrsf] Saved PLY: {path}")


# -------------
# Main routine
# -------------
def main():
    rospy.init_node("limrsf_mesh_pipeline_node")

    # ROS publishers (publish AFTER simplification)
    mesh_pub = rospy.Publisher("/mesh_data", CustomMesh, queue_size=10)
    ply_pub = rospy.Publisher("/mesh_ply_file", String, queue_size=10)

    # I/O
    pcd_file = rospy.get_param("~pcd_file", os.path.expanduser("~/LiMRSF/meiclab1.pcd"))
    out_dir = rospy.get_param("~out_dir", os.path.expanduser("~/LiMRSF"))
    os.makedirs(out_dir, exist_ok=True)
    highlighted_ply = os.path.join(out_dir, "highlighted_mesh.ply")
    simplified_ply  = os.path.join(out_dir, "simplified_mesh.ply")
    indices_npy     = os.path.join(out_dir, "mesh_blind_spot_indices.npy")

    # Preprocessing params
    nb_neighbors   = int(rospy.get_param("~nb_neighbors", 20))
    std_ratio      = float(rospy.get_param("~std_ratio", 2.0))
    normal_radius  = float(rospy.get_param("~normal_radius", 0.5))  # 0.5
    normal_max_nn  = int(rospy.get_param("~normal_max_nn", 30))
    poisson_depth  = int(rospy.get_param("~poisson_depth", 12))

    # RGB transfer
    knn = int(rospy.get_param("~knn", 3))
    saturation_boost = float(rospy.get_param("~saturation_boost", 1.0))

    # Blind-spot thresholding (baseline switch)
    threshold_mode = rospy.get_param("~threshold_mode", "relative")  # {"relative","percentile"}
    tau        = float(rospy.get_param("~tau", 0.7))
    percentile = float(rospy.get_param("~percentile", 40.0))
    min_component_size = int(rospy.get_param("~min_component_size", 50))

    # Simplification
    target_tris = int(rospy.get_param("~target_tris", 10000))

    # ----------------
    # Load point cloud
    # ----------------
    if not os.path.exists(pcd_file):
        raise FileNotFoundError(f"Point cloud not found: {pcd_file}")
    pcd = o3d.io.read_point_cloud(pcd_file)

    t0 = time.perf_counter()

    # 1) Outlier removal
    t = time.perf_counter()
    pcd = remove_outliers(pcd, nb_neighbors, std_ratio)
    t_outlier = time.perf_counter() - t

    # 2) Normal estimation
    t = time.perf_counter()
    normal_max_nn = min(normal_max_nn, 24)
    pcd = compute_normals(pcd, normal_radius, normal_max_nn)
    t_normals = time.perf_counter() - t

    # 3) Poisson reconstruction
    t = time.perf_counter()
    mesh, densities = poisson_reconstruction(pcd, depth=poisson_depth)
    t_poisson = time.perf_counter() - t

    # 4) RGB transfer (RGB-textured mesh)
    t = time.perf_counter()
    mesh = kdtree_rgb_transfer(pcd, mesh, k=knn, saturation_boost=saturation_boost)
    t_rgb = time.perf_counter() - t

    # 5) Low-density mask
    t = time.perf_counter()
    mask = compute_low_density_mask(densities, mode=threshold_mode, tau=tau, percentile=percentile)

    # 6) Small-component filtering
    mask = filter_small_components(mask, mesh, min_size=min_component_size)

    # 저장: 인덱스(.npy) — 변수명/경로 수정 (output_dir→out_dir, blind_mask→mask)
    np.save(indices_npy, np.where(np.asarray(mask))[0])
    rospy.loginfo(f"[limrsf] Saved blind-spot indices: {indices_npy}")

    # Colourise mask (red); save highlighted mesh
    mesh = paint_mask_on_mesh(mesh, mask, colour_low=(1.0, 0.0, 0.0))
    t_mask = time.perf_counter() - t

    # 7) Save pre-simplified highlighted mesh
    save_ply(mesh, highlighted_ply)

    # 8) (helper) mesh_to_ros_msg()

    # 9) Simplify mesh (QEM)
    t = time.perf_counter()
    simp = simplify_mesh_qem(mesh, target_tris=target_tris)
    t_simplify = time.perf_counter() - t

    # 10) Save simplified mesh
    save_ply(simp, simplified_ply)

    # 11) Publish for Unity (ROS-TCP): path + mesh (simplified)
    ply_pub.publish(simplified_ply)
    mesh_pub.publish(mesh_to_ros_msg(simp))

    # Timing summary
    t_total = time.perf_counter() - t0
    rospy.loginfo("[limrsf] timings "
                  f"| outlier={t_outlier:.3f}s normals={t_normals:.3f}s "
                  f"poisson={t_poisson:.3f}s rgb={t_rgb:.3f}s mask+filter={t_mask:.3f}s "
                  f"simplify={t_simplify:.3f}s TOTAL={t_total:.3f}s")
    rospy.loginfo("[limrsf] Pipeline complete.")
    rospy.sleep(0.5)  # allow pubs to flush

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr(f"[limrsf] Exception: {e}")
        raise
