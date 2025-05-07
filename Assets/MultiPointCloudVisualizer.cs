using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Microsoft.MixedReality.Toolkit.UI;  // For ObjectManipulator
using Microsoft.MixedReality.Toolkit.Input;  // For hand interaction components

public class MultiPointCloudVisualizer : MonoBehaviour
{
    ROSConnection ros;
    public List<string> pointCloudTopics = new List<string>
    {
        "/RGB_map_0",
        "/RGB_map_1",
        "/RGB_map_2",
    };

    public string transformTopic = "/hl2_to_flir_transform";
    public int maxPoints = 100000;
    public float pointSize = 0.02f;

    private Vector3 translation;
    private Quaternion rotation;
    private bool transformReceived = false;

    private Dictionary<string, ParticleSystem.Particle[]> topicParticles = new Dictionary<string, ParticleSystem.Particle[]>();
    private Dictionary<string, ParticleSystem> particleSystems = new Dictionary<string, ParticleSystem>();

    // Custom initial rotation to correct FLIR -> Unity orientation
    public Quaternion initialRotationCorrection = Quaternion.Euler(-90f, 0f, 0f);

    // To track the interaction components
    private ObjectManipulator objectManipulator;
    private BoxCollider boxCollider;

    void Start()
    {
        // Load the PointCloudMaterial from Resources
        Material pointCloudMaterial = Resources.Load<Material>("PointCloudMaterial");

        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TransformStampedMsg>(transformTopic, UpdateTransform);

        foreach (var topic in pointCloudTopics)
        {
            ros.Subscribe<PointCloud2Msg>(topic, msg => UpdatePointCloud(msg, topic));

            // Initialize Particle System for each topic
            var particleSystem = new GameObject(topic).AddComponent<ParticleSystem>();
            particleSystem.transform.parent = this.transform;

            var main = particleSystem.main;
            main.loop = false;
            main.playOnAwake = false;
            main.maxParticles = maxPoints;

            var emission = particleSystem.emission;
            emission.enabled = false;

            var shape = particleSystem.shape;
            shape.enabled = false;

            // Assign the material to the ParticleSystemRenderer
            var renderer = particleSystem.GetComponent<ParticleSystemRenderer>();
            renderer.material = pointCloudMaterial;

            particleSystems[topic] = particleSystem;
            topicParticles[topic] = new ParticleSystem.Particle[maxPoints];
        }

        // Add interaction components to the parent (this object)
        AddInteractionComponents(this.gameObject);
    }

    void AddInteractionComponents(GameObject pointCloudObject)
    {
        // Add ObjectManipulator for hand manipulation and track it
        objectManipulator = pointCloudObject.AddComponent<ObjectManipulator>();
        objectManipulator.OnManipulationEnded.AddListener(ResetInteraction);  // Listen for manipulation end event

        // Add NearInteractionGrabbable for near-hand interaction
        pointCloudObject.AddComponent<NearInteractionGrabbable>();

        // Add a Collider so the point cloud can be grabbed and track it
        boxCollider = pointCloudObject.AddComponent<BoxCollider>();

        // Adjust the collider size as needed
        boxCollider.size = new Vector3(1.0f, 1.0f, 1.0f);  // Adjust this size based on the estimated cloud size
    }

    // Callback when the manipulation ends to reset the object state
    void ResetInteraction(ManipulationEventData data)
    {
        // Re-enable the collider in case it got disabled or changed during manipulation
        boxCollider.enabled = false;
        boxCollider.enabled = true;

        // Optionally re-enable ObjectManipulator if it was disabled
        objectManipulator.enabled = false;
        objectManipulator.enabled = true;

        Debug.Log("Interaction Reset Complete");
    }

    void UpdateTransform(TransformStampedMsg transformMsg)
    {
        // Extract translation from FLIR to HL2
        translation = new Vector3(
            (float)transformMsg.transform.translation.x,
            (float)transformMsg.transform.translation.y,
            (float)transformMsg.transform.translation.z);

        rotation = new Quaternion(
            (float)transformMsg.transform.rotation.x,
            (float)transformMsg.transform.rotation.y,
            (float)transformMsg.transform.rotation.z,
            (float)transformMsg.transform.rotation.w);

        // Apply initial rotation correction for left-handed to right-handed conversion
        rotation = initialRotationCorrection * rotation;

        // Normalize quaternion
        rotation.Normalize();
        transformReceived = true;

        Debug.Log("Received Translation: " + translation);
        Debug.Log("Received Rotation: " + rotation.eulerAngles);
    }

    void UpdatePointCloud(PointCloud2Msg pointCloud, string topic)
    {
        if (!transformReceived)
        {
            Debug.LogWarning("Transform not received yet.");
            return;
        }

        if (pointCloud == null || pointCloud.data == null || pointCloud.data.Length == 0)
        {
            Debug.LogError("Invalid point cloud data for topic: " + topic);
            return;
        }

        int pointStep = (int)pointCloud.point_step;
        int numPoints = Mathf.Min(maxPoints, (int)(pointCloud.data.Length / pointStep));
        var particles = topicParticles[topic];

        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * pointStep;

            float x = BitConverter.ToSingle(pointCloud.data, offset + (int)pointCloud.fields[0].offset);
            float y = BitConverter.ToSingle(pointCloud.data, offset + (int)pointCloud.fields[1].offset);
            float z = BitConverter.ToSingle(pointCloud.data, offset + (int)pointCloud.fields[2].offset);

            // Apply transformation (FLIR to HL2), including rotation correction
            Vector3 point = new Vector3(x, y, z);
            point = rotation * point + translation;

            // Adjust the point's position to match Unity's left-handed coordinate system (flip Z-axis)
            point = new Vector3(point.x, point.y, -point.z);

            // Extract color data
            byte r = (byte)pointCloud.data[offset + (int)pointCloud.fields[3].offset + 2];
            byte g = (byte)pointCloud.data[offset + (int)pointCloud.fields[3].offset + 1];
            byte b = (byte)pointCloud.data[offset + (int)pointCloud.fields[3].offset];
            Color32 color = new Color32(r, g, b, 255);

            particles[i].position = point;
            particles[i].startColor = color;
            particles[i].startSize = pointSize;
        }

        // Disable unused points
        for (int i = numPoints; i < maxPoints; i++)
        {
            particles[i].startSize = 0f;
        }

        // Set updated particles in the particle system
        particleSystems[topic].SetParticles(particles, particles.Length);
    }
}