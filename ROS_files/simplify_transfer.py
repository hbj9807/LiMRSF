import open3d as o3d
import numpy as np
import os
import rospy
from std_msgs.msg import String
from mesh_publisher.msg import CustomMesh
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Function to simplify the mesh
def simplify_mesh(mesh, target_number_of_triangles=100000):
    print(f"Simplifying mesh to {target_number_of_triangles} triangles...")
    simplified_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_number_of_triangles)
    print(f"Simplified mesh has {len(simplified_mesh.triangles)} triangles.")
    return simplified_mesh

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

def save_mesh_as_ply(mesh, file_name):
    print(f"Saving mesh as {file_name}")
    o3d.io.write_triangle_mesh(file_name, mesh)

def main():
    rospy.init_node('mesh_publisher')

    # Publisher for PLY file path
    ply_pub = rospy.Publisher('/mesh_ply_file', String, queue_size=10)
    
    # Publisher for the custom mesh message
    mesh_pub = rospy.Publisher('/mesh_data', CustomMesh, queue_size=10)

    # Load previously saved highlighted mesh
    highlighted_mesh_file = os.path.expanduser("~/r3live_output/highlighted_mesh.ply")
    if not os.path.exists(highlighted_mesh_file):
        raise FileNotFoundError(f"Highlighted mesh file {highlighted_mesh_file} does not exist!")

    mesh = o3d.io.read_triangle_mesh(highlighted_mesh_file)
    print(f"Loaded highlighted mesh: {highlighted_mesh_file}")

    # Simplify the mesh
    simplified_mesh = simplify_mesh(mesh, target_number_of_triangles=10000)  # Store simplified mesh in simplified_mesh

    # Save the simplified mesh as a PLY file
    save_mesh_as_ply(simplified_mesh, os.path.expanduser("~/r3live_output/simplified_mesh.ply"))

    # Publish the simplified mesh data to Unity via ROS
    save_and_publish_mesh(mesh_pub, simplified_mesh)  # Make sure the simplified mesh is sent

if __name__ == "__main__":
    main()
