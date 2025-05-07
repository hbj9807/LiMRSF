import open3d as o3d
import numpy as np
import os
import rospy
from std_msgs.msg import String
from mesh_publisher.msg import CustomMesh
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Functions for processing point clouds and meshes
def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    print("Removing outliers...")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    inlier_cloud = pcd.select_by_index(ind)
    return inlier_cloud

def compute_normals(pcd, radius=0.5):
    print("Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(100)
    return pcd

def crop_point_cloud(pcd):
    print("Cropping point cloud...")
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-10, -10, -5), max_bound=(10, 10, 5))
    pcd = pcd.crop(bbox)
    return pcd

def perform_poisson_reconstruction(pcd, depth=13):
    print("Performing Poisson surface reconstruction")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    return mesh, densities

def highlight_blind_spots(mesh, densities, density_threshold=0.3):
    print("Highlighting blind spots...")
    mean_density = np.mean(densities)
    threshold = mean_density * density_threshold

    # Initialize color array
    vertex_colors = np.asarray(mesh.vertex_colors) if len(mesh.vertex_colors) > 0 else np.ones((len(mesh.vertices), 3))

    # Identify low-density vertices (blind spots)
    low_density_mask = densities < threshold
    low_density_indices = np.where(low_density_mask)[0]

    # Highlight low-density vertices in red (blind spots)
    vertex_colors[low_density_mask] = [1, 0, 0]  # Red for blind spots

    # Ensure that the rest of the vertices retain their original colors
    mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    return mesh, low_density_indices  # Return both mesh and indices

def transfer_point_cloud_colors_to_mesh(pcd, mesh):
    """Transfer colors from the point cloud directly to the mesh."""
    print("Transferring colors from point cloud to mesh...")

    # Ensure that the point cloud has colors
    if len(pcd.colors) == 0:
        raise ValueError("Point cloud does not contain colors.")

    # Normalize colors to [0, 1] if not already
    pcd_colors = np.asarray(pcd.colors)
    pcd_points = np.asarray(pcd.points)

    # Find the nearest points from the point cloud to the mesh vertices
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    mesh_vertex_colors = np.zeros((len(mesh.vertices), 3))

    for i, vertex in enumerate(mesh.vertices):
        _, idx, _ = kdtree.search_knn_vector_3d(vertex, 3)  # Using 3 nearest neighbors
        nearest_colors = pcd_colors[idx]
        mesh_vertex_colors[i] = np.mean(nearest_colors, axis=0)  # Averaging the colors

    # Optional: Enhance color saturation
    mesh_vertex_colors = np.clip(mesh_vertex_colors * 1.2, 0, 1)  # Boost saturation

    # Apply colors to mesh
    mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_vertex_colors)
    return mesh

# Removed the simplify_mesh function as it's no longer needed

# Function to convert Open3D mesh to custom ROS message format
def mesh_to_ros_msg(mesh):
    """Convert Open3D mesh to the custom ROS message format."""
    custom_mesh_msg = CustomMesh()

    # Convert vertices to geometry_msgs/Point
    vertices = np.asarray(mesh.vertices)
    custom_mesh_msg.vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in vertices]

    # Convert triangles to uint32 array
    triangles = np.asarray(mesh.triangles)
    custom_mesh_msg.triangles = triangles.flatten().tolist()

    # Convert vertex colors to std_msgs/ColorRGBA
    if len(mesh.vertex_colors) > 0:
        colors = np.asarray(mesh.vertex_colors)
        custom_mesh_msg.vertex_colors = [ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0) for c in colors]
    
    return custom_mesh_msg

# Function to save and publish mesh in ROS format
def save_and_publish_mesh(ros_pub, mesh):
    # Convert the mesh to a ROS message
    custom_mesh_msg = mesh_to_ros_msg(mesh)
    
    # Log mesh information for debugging
    print(f"Publishing mesh with {len(custom_mesh_msg.vertices)} vertices and {len(custom_mesh_msg.triangles)} triangles.")
    
    # Publish the mesh message
    ros_pub.publish(custom_mesh_msg)
    rospy.loginfo("Published custom mesh message to ROS.")

def save_and_publish_ply(ros_pub, mesh):
    # Save the mesh as a .ply file
    ply_file_path = os.path.expanduser("~/r3live_output/mesh_result_with_blindspots.ply")
    o3d.io.write_triangle_mesh(ply_file_path, mesh)
    print(f"Mesh saved to: {ply_file_path}")

def main():
    rospy.init_node('mesh_publisher')

    # Publisher for PLY file path
    ply_pub = rospy.Publisher('/mesh_ply_file', String, queue_size=10)
    
    # Publisher for the custom mesh message
    mesh_pub = rospy.Publisher('/mesh_data', CustomMesh, queue_size=10)

    # Load point cloud
    pcd_file = os.path.expanduser("~/r3live_output/meic_scan.pcd")
    if not os.path.exists(pcd_file):
        raise FileNotFoundError(f"Point cloud file {pcd_file} does not exist!")

    pcd = o3d.io.read_point_cloud(pcd_file)
    print(f"Loaded point cloud: {pcd_file}")

    # Remove outliers, crop, and compute normals
    pcd = remove_outliers(pcd, nb_neighbors=50, std_ratio=1.0)
    pcd = crop_point_cloud(pcd)
    pcd = compute_normals(pcd, radius=0.7)

    # Perform Poisson surface reconstruction
    mesh, densities = perform_poisson_reconstruction(pcd, depth=12)

    # Transfer the original point cloud colors to the mesh
    mesh = transfer_point_cloud_colors_to_mesh(pcd, mesh)

    # Highlight blind spots based on density
    mesh, low_density_indices = highlight_blind_spots(mesh, densities, density_threshold=0.7)

    # Save the blind spot indices to a .npy file
    blind_spot_indices_file = 'mesh_blind_spot_indices.npy'
    np.save(blind_spot_indices_file, low_density_indices)
    print(f"Saved blind spot indices to {blind_spot_indices_file}")

    # Removed mesh simplification process

    # Save and publish the PLY file path
    save_and_publish_ply(ply_pub, mesh)

    # Publish the mesh data to Unity via ROS
    save_and_publish_mesh(mesh_pub, mesh)

if __name__ == "__main__":
    main()