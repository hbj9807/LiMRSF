using UnityEngine;
using System.Net.Sockets;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Threading.Tasks;

public class Hl2ssCameraStreamer : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    private byte[] buffer;
    private ROSConnection ros;

    // HoloLens 2 Server Details
    public string hl2ssServerIP = "192.168.1.161";  // Replace with your HoloLens 2 IP
    public int hl2ssPort = 3800;  // The port you configured for hl2ss

    // Unity Texture
    public Texture2D cameraTexture;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Start connection to hl2ss server
        ConnectToHl2ssServer(hl2ssServerIP, hl2ssPort);
    }

    void ConnectToHl2ssServer(string ip, int port)
    {
        client = new TcpClient(ip, port);
        stream = client.GetStream();
        Debug.Log("Connected to hl2ss server.");

        // Handle incoming data
        Task.Run(() => ListenForData());
    }

    async void ListenForData()
    {
        while (true)
        {
            if (stream.DataAvailable)
            {
                buffer = new byte[client.ReceiveBufferSize];
                int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    ProcessImage(buffer, bytesRead);
                }
            }
        }
    }

    void ProcessImage(byte[] data, int length)
    {
        // Decode JPEG data into a Texture2D
        if (cameraTexture == null)
            cameraTexture = new Texture2D(1920, 1080, TextureFormat.RGB24, false);

        cameraTexture.LoadImage(data);

        // Publish the image to ROS
        PublishToROS(data);
    }

    void PublishToROS(byte[] imageData)
    {
        CompressedImageMsg rosImage = new CompressedImageMsg
        {
            header = new HeaderMsg
            {
                stamp = GetROSTime(),
                frame_id = "hl2_camera"
            },
            format = "jpeg",
            data = imageData
        };

        ros.Send("/HL2/compressed", rosImage);
    }

    private TimeMsg GetROSTime()
    {
        double epochTime = Time.timeSinceLevelLoad;
        uint secs = (uint)epochTime;
        uint nsecs = (uint)((epochTime - secs) * 1e9);

        return new TimeMsg
        {
            sec = secs,
            nanosec = nsecs
        };
    }

    void OnApplicationQuit()
    {
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
    }
}