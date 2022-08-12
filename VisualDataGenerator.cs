using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class VisualDataGenerator : MonoBehaviour
{
    public GameObject _camera;
    public Camera _Camera;

    int sample_freq;
    int c = 1;
    int n = 0;
    int i = 0;
    int l = 0;

    //User-defined parameters//
    //-----------------------//
    //Define dataset: 'TUM' if using TUM VI or 'ST' if using SenseTime
    static string dataset = "ST";
    //Define sequence to be used (e.g., 'A1')
    static string sequence = "A1";
    //Define name of virtual environment experiment (name of folder) 
    static string experiment = "A1_Name";
    //Define frame rate of output camera 
    static int camera_frame_rate = 20;
    //Define camera horizontal field of view
    float HFoV = 79.0f;
    //Define camera image width
    float camera_width = 640.0f;
    //Define camera image height
    float camera_height = 480.0f;
    //Define number of new camera poses (based on 20Hz frame rate: room5=2834, A1=2409, A4=1977, A6=1458)
    static int length = 2409;
    //Define set of light levels to test
    float[] light_levels = new float[] { 50.0f, 100.0f, 200.0f, 300.0f, 400.0f, 550.0f, 750.0f, 1000.0f, 2500.0f, 5000.0f };
    //Define path to the GT csv file
    string csv = "D:/SharedData/" + sequence + "_gt.csv";
    //Define location of experiment folders
    string experiments_folder = "D:/SharedData/";
    //-----------------------//

    //to hold new pose data
    string output = "";
    string[] timestamps = new string[length];
    float[] posx = new float[length];
    float[] posy = new float[length];
    float[] posz = new float[length];
    float[] quatw = new float[length];
    float[] quatx = new float[length];
    float[] quaty = new float[length];
    float[] quatz = new float[length];

    public GameObject _light1;
    //variable where we will store the HD lighting data
    private HDAdditionalLightData lightData;

    // Start is called before the first frame update
    void Start()
    {
        //target frame rate for *rendering* frames (can be adjusted according to computational resources available)
        Application.targetFrameRate = 20;

        //set up access to light level in HDRP
        lightData = _light1.GetComponent<HDAdditionalLightData>();
        
        //read in ground truth csv file
        string[] gt = File.ReadAllLines(csv);
        //create array list copy
        var gt2 = new ArrayList(gt);


        //Sample ground truth to create camera poses//
        //------------------------------------------//
        
        //sample frequency is original ground truth frequency (TUM VI:120Hz, SenseTime:400Hz) divided by desired camera frame rate
        if (dataset == "TUM")
        {
            sample_freq = 120 / camera_frame_rate;
        }
        else if (dataset == "ST")
        {
            sample_freq = 400 / camera_frame_rate;
        }
        c = 1;

        //Debug.Log(gt2.Count);

        foreach (string line in gt2)
        {
            //get every nth row of csv, skipping 1st row (change c +/- number to start on different row)
            if ((c != 1) && (c + 4) % sample_freq == 0)
            {
                string[] data = line.Split(',');
                timestamps[n] = data[0];
                posx[n] = float.Parse(data[1]);
                posy[n] = float.Parse(data[2]);
                posz[n] = float.Parse(data[3]);
                quatw[n] = float.Parse(data[4]);
                quatx[n] = float.Parse(data[5]);
                quaty[n] = float.Parse(data[6]);
                quatz[n] = float.Parse(data[7]);
                //increment array counter
                n += 1;
            }
            //increment csv line counter
            c += 1;
        }

        //Write config file (ORB-SLAM3 version)
        string yamlOut = @experiments_folder + "Unity_" + dataset + ".yaml";
        // Create a new file     
        using (StreamWriter outputFile = new StreamWriter(yamlOut))
        {
            outputFile.WriteLine("%YAML:1.0");
            outputFile.WriteLine("#Camera Intrinsics");
            outputFile.WriteLine("Camera.type: \"PinHole\"");
            outputFile.WriteLine("Camera.fx: " + (camera_width / (2*(Math.Tan((HFoV/2.0f)*(Math.PI/180))))).ToString("000.0"));
            outputFile.WriteLine("Camera.fy: " + (camera_width / (2*(Math.Tan((HFoV/2.0f)*(Math.PI/180))))).ToString("000.0"));
            outputFile.WriteLine("Camera.cx: " + ((camera_width - 1) / 2).ToString());
            outputFile.WriteLine("Camera.cy: " + ((camera_height - 1) / 2).ToString());
            outputFile.WriteLine("Camera.k1: 0.0");
            outputFile.WriteLine("Camera.k2: 0.0");
            outputFile.WriteLine("Camera.p1: 0.0");
            outputFile.WriteLine("Camera.p2: 0.0");
            outputFile.WriteLine("Camera.width: " + ((int)camera_width).ToString());
            outputFile.WriteLine("Camera.height: " + ((int)camera_height).ToString());
            outputFile.WriteLine("Camera.fps: " + (camera_frame_rate).ToString() + ".0");
            outputFile.WriteLine("Camera.RGB: 1");
            outputFile.WriteLine("");
            outputFile.WriteLine("#Camera Extrinsics");
            outputFile.WriteLine("Tbc: !!opencv-matrix");
            outputFile.WriteLine("   rows: 4");
            outputFile.WriteLine("   cols: 4");
            outputFile.WriteLine("   dt: f");
            if (dataset == "TUM")
            {
                outputFile.WriteLine("   data: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]");
            }
            else if (dataset == "ST")
            {
                outputFile.WriteLine("   data: [0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]");
            } 
            outputFile.WriteLine("");
            outputFile.WriteLine("#IMU noise");
            if (dataset == "TUM")
            {
                outputFile.WriteLine("IMU.NoiseGyro: 0.00016");
                outputFile.WriteLine("IMU.NoiseAcc: 0.0028");
                outputFile.WriteLine("IMU.GyroWalk: 000022");
                outputFile.WriteLine("IMU.AccWalk: 0.00086");
                outputFile.WriteLine("IMU.Frequency: 200");
            }
            else if (dataset == "ST")
            {
                outputFile.WriteLine("IMU.NoiseGyro: 0.0025");
                outputFile.WriteLine("IMU.NoiseAcc: 0.025");
                outputFile.WriteLine("IMU.GyroWalk: 8e-04");
                outputFile.WriteLine("IMU.AccWalk: 0.02");
                outputFile.WriteLine("IMU.Frequency: 400");
            }
            outputFile.WriteLine("");
            outputFile.WriteLine("#ORB Extractor");
            outputFile.WriteLine("ORBextractor.nFeatures: 1000");
            outputFile.WriteLine("ORBextractor.scaleFactor: 1.2");
            outputFile.WriteLine("ORBextractor.nLevels: 8");
            outputFile.WriteLine("ORBextractor.iniThFAST: 20");
            outputFile.WriteLine("ORBextractor.minThFAST: 7");
            outputFile.WriteLine("");
            outputFile.WriteLine("#Viewer");
            outputFile.WriteLine("Viewer.KeyFrameSize: 0.05");
            outputFile.WriteLine("Viewer.KeyFrameLineWidth: 1");
            outputFile.WriteLine("Viewer.GraphLineWidth: 0.9");
            outputFile.WriteLine("Viewer.PointSize: 2");
            outputFile.WriteLine("Viewer.CameraSize: 0.08");
            outputFile.WriteLine("Viewer.CameraLineWidth: 3");
            outputFile.WriteLine("Viewer.ViewpointX: 0");
            outputFile.WriteLine("Viewer.ViewpointY: -0.7");
            outputFile.WriteLine("Viewer.ViewpointZ: -3.5");
            outputFile.WriteLine("Viewer.ViewpointF: 500");
        }

        //Write timestamps to text fileas (required by ORB-SLAM3)
        string txtOut = @experiments_folder + experiment.ToString() + "/InputData/" + sequence + "/" + sequence + ".txt";
        // Create a new file     
        using (StreamWriter outputFile = new StreamWriter(txtOut))
        {
            foreach (string timestamp in timestamps)
            {
                outputFile.WriteLine(timestamp);
            }
        }

        //Write timestamps to csv file (required by ORB-SLAM3)
        string csvOut = @experiments_folder + experiment.ToString() + "/InputData/" + sequence + "/mav0/cam0/data.csv";
        //Create a new file     
        using (StreamWriter outputFile = new StreamWriter(csvOut))
        {
            outputFile.WriteLine("#timestamp [ns],filename");
            foreach (string timestamp in timestamps)
            {
                string csvLine = timestamp + "," + timestamp + ".png";
                outputFile.WriteLine(csvLine);
            }
        }

        Debug.Log("Camera poses prepared from groundtruth");

        //Apply start settings
        lightData.intensity = light_levels[l];
        Debug.Log("Light intensity set to " + light_levels[l].ToString());
        output = experiments_folder + experiment.ToString() + "/InputData/" + sequence + "_" + l.ToString() + "/mav0/cam0/data/";
    }

    // Update is called once per frame
    void Update()
    {
        //Create camera images//
        //--------------------//

        if (i < length && l < light_levels.Length)
        {
            //Set camera pose transformation - SenseTime requires additional rotation
            if (dataset == "TUM")
            {
                //TUM VI
                _camera.transform.position = new Vector3(posx[i] * -1.0f, posz[i] * 1.0f, posy[i] * -1.0f);
                _camera.transform.rotation = new Quaternion(quatx[i] * 1.0f, quatz[i] * -1.0f, quaty[i] * 1.0f, quatw[i]);
            }
            else if (dataset == "ST")
            {
                //SenseTime
                Quaternion st_rotation = Quaternion.Euler(90, 90, 180);
                _camera.transform.position = new Vector3(posx[i] * -1.0f, posz[i] * 1.0f, posy[i] * -1.0f);
                _camera.transform.rotation = new Quaternion(quatx[i] * 1.0f, quatz[i] * -1.0f, quaty[i] * 1.0f, quatw[i] * 1.0f) * st_rotation;
            }

            //Capture camera texture
            RenderTexture activeRenderTexture = RenderTexture.active;
            RenderTexture.active = _Camera.targetTexture;
            _Camera.Render();
            Texture2D tempTexture = new Texture2D(_Camera.targetTexture.width, _Camera.targetTexture.height, TextureFormat.ARGB32, false, true);
            tempTexture.ReadPixels(new Rect(0, 0, _Camera.targetTexture.width, _Camera.targetTexture.height), 0, 0);
            tempTexture.Apply();
            RenderTexture.active = activeRenderTexture;

            //Save using another texture
            Color[] colorSrc = tempTexture.GetPixels(0, 0, _Camera.targetTexture.width, _Camera.targetTexture.height);
            Texture2D outTexture = new Texture2D(_Camera.targetTexture.width, _Camera.targetTexture.height, TextureFormat.R8, false, true);
            outTexture.SetPixels(colorSrc);
            outTexture.Apply(true, false);
            byte[] bytes = outTexture.EncodeToPNG();
            Destroy(tempTexture);
            Destroy(outTexture);
            File.WriteAllBytes(output + timestamps[i] + ".png", bytes);

            //Progress log
            Debug.Log("Light level: " + l + ", Image: " + i);
            i += 1;
        }
        //If all light levels done, complete
        else if (i == length && l == light_levels.Length - 1)
        {
            Debug.Log("Visual data generation complete");
        }
        //If not, set up next light level and continue
        else if (i == length && l < light_levels.Length - 1)
        {
            l += 1;
            lightData.intensity = light_levels[l];
            Debug.Log("Light intensity set to " + light_levels[l].ToString());
            output = experiments_folder + experiment.ToString() + "/InputData/" + sequence + "_" + l.ToString() + "/mav0/cam0/data/";
            i = 0;
        }
    }
}
