using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.MeshPublisher;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Input;

public class MeshReceiver : MonoBehaviour
{
    ROSConnection ros;

    [Tooltip("Topic to subscribe to for receiving mesh data.")]
    public string topic = "/mesh_data";

    [Tooltip("Material to apply to the mesh. This can be set from the Unity Editor.")]
    public Material meshMaterial;  // Exposed field for material selection

    [Tooltip("Scaling factor for the received mesh.")]
    public float scaleFactor = 1.0f;  // Exposed field for scaling

    private ObjectManipulator objectManipulator;
    private BoxCollider meshCollider;

    void Start()
    {
        // Initialize the ROS connection and subscribe to the topic
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CustomMeshMsg>(topic, ReceiveMesh);
    }

    void ReceiveMesh(CustomMeshMsg meshMsg)
    {
        Debug.Log($"Received mesh with {meshMsg.vertices.Length} vertices and {meshMsg.triangles.Length} triangles.");

        // Ensure the triangle array length is divisible by 3 for proper triangulation
        if (meshMsg.triangles.Length % 3 != 0)
        {
            Debug.LogError("Triangle array length is not divisible by 3. The mesh might not be properly triangulated.");
            return;
        }

        // Create a new Unity Mesh
        Mesh mesh = new Mesh();

        // Convert vertices with scaling and coordinate inversion (inverting X axis for left-right correction)
        Vector3[] vertices = new Vector3[meshMsg.vertices.Length];
        for (int i = 0; i < meshMsg.vertices.Length; i++)
        {
            vertices[i] = new Vector3(
                -(float)meshMsg.vertices[i].x * scaleFactor,  // Invert the X-axis
                (float)meshMsg.vertices[i].y * scaleFactor,
                (float)meshMsg.vertices[i].z * scaleFactor
            );
        }

        // Convert triangles and fix the winding order
        int[] triangles = new int[meshMsg.triangles.Length];
        for (int i = 0; i < meshMsg.triangles.Length; i += 3)
        {
            // Reverse winding order for proper face orientation
            triangles[i] = (int)meshMsg.triangles[i];
            triangles[i + 1] = (int)meshMsg.triangles[i + 2];
            triangles[i + 2] = (int)meshMsg.triangles[i + 1];
        }

        // Log some of the triangle indices for debugging
        for (int i = 0; i < Mathf.Min(9, triangles.Length); i++)  // Log the first 3 triangles
        {
            Debug.Log($"Triangle index {i}: {triangles[i]}");
        }

        // Assign vertices and triangles to the Unity mesh
        mesh.vertices = vertices;
        mesh.triangles = triangles;

        // Recalculate normals to ensure smooth shading
        mesh.RecalculateNormals();

        // Check and apply vertex colors
        if (meshMsg.vertex_colors.Length > 0)
        {
            Color[] colors = new Color[meshMsg.vertex_colors.Length];
            for (int i = 0; i < meshMsg.vertex_colors.Length; i++)
            {
                colors[i] = new Color(
                    meshMsg.vertex_colors[i].r,
                    meshMsg.vertex_colors[i].g,
                    meshMsg.vertex_colors[i].b,
                    meshMsg.vertex_colors[i].a
                );
            }
            mesh.colors = colors;
        }

        // Ensure there is a MeshFilter and MeshRenderer on this GameObject
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null)
        {
            meshFilter = gameObject.AddComponent<MeshFilter>();
        }

        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        if (meshRenderer == null)
        {
            meshRenderer = gameObject.AddComponent<MeshRenderer>();
        }

        // Apply the selected material from the Unity Editor, or use a default material if none is assigned
        if (meshMaterial != null)
        {
            meshRenderer.material = meshMaterial;
        }
        else
        {
            Debug.LogWarning("No material assigned in the Unity Editor. Assign a material to render the mesh properly.");
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        // Assign the generated mesh to the MeshFilter
        meshFilter.mesh = mesh;

        Debug.Log("Mesh assigned and rendered.");

        // Debug the mesh bounds for additional feedback
        Debug.Log($"Mesh bounds: {mesh.bounds.size}");

        // Add the ObjectManipulator to allow the user to grab and rotate/move the mesh
        AddObjectManipulator();

        // Add a BoxCollider for interaction with the mesh
        AddCollider(mesh.bounds.size);
    }

    // Adds the ObjectManipulator component from MRTK to allow object manipulation
    private void AddObjectManipulator()
    {
        objectManipulator = gameObject.GetComponent<ObjectManipulator>();
        if (objectManipulator == null)
        {
            objectManipulator = gameObject.AddComponent<ObjectManipulator>();
            objectManipulator.SmoothingFar = true;
            objectManipulator.OnManipulationStarted.AddListener(OnManipulationStarted);
            objectManipulator.OnManipulationEnded.AddListener(OnManipulationEnded);
        }

        // Enable near and far interaction for both direct hand touches and remote pointer
        NearInteractionGrabbable nearInteractionGrabbable = gameObject.GetComponent<NearInteractionGrabbable>();
        if (nearInteractionGrabbable == null)
        {
            gameObject.AddComponent<NearInteractionGrabbable>();
        }
    }

    // Add a BoxCollider to the mesh to enable interaction
    private void AddCollider(Vector3 size)
    {
        meshCollider = gameObject.GetComponent<BoxCollider>();
        if (meshCollider == null)
        {
            meshCollider = gameObject.AddComponent<BoxCollider>();
        }

        // Adjust the size of the collider to fit the mesh bounds
        meshCollider.size = size;
    }

    // Event handler when manipulation starts
    private void OnManipulationStarted(ManipulationEventData data)
    {
        Debug.Log("Object manipulation started.");
    }

    // Event handler when manipulation ends
    private void OnManipulationEnded(ManipulationEventData data)
    {
        Debug.Log("Object manipulation ended.");
    }
}