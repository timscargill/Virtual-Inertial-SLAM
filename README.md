# Virtual-Inertial-SLAM

Virtual-Inertial SLAM is a game engine-based emulator for running visual-inertial simultaneous localization and mapping (VI-SLAM) in virtual environments with real inertial data. Virtual visual data (camera images) are generated in the Unity game engine, and combined with the inertial data from existing SLAM datasets, preserving access to ground truth pose data. These source data can then be input to a VI-SLAM algorithm such as ORB-SLAM3, as illustrated below:

_Virtual-Inertial-SLAM in action in a virtual museum environment, using ORB-SLAM3 and the SenseTime dataset (camera image is oriented this way due to orientation of the smartphone device used in this dataset):_
![GIF of Virtual-Inertial-SLAM in action](https://github.com/timscargill/Virtual-Inertial-SLAM/blob/main/Virtual-Inertial-SLAM.gif?raw=true)

This repository accompanies the the paper '[_Integrated design of augmented reality spaces using virtual environments_](https://ieeexplore.ieee.org/document/9995481)', in Proceedings of IEEE ISMAR `22. Examples of our other projects which use Virtual-Inertial SLAM include [Invisible Textures](https://github.com/timscargill/Invisible-Textures/) and [SiTAR](https://github.com/timscargill/SiTAR/).

# Resources

This repository contains the following resources (see below for implementation instructions):

1) Example directory structure (_A1_Name.zip_): A sample of the directory structure which we use to implement our VI-SLAM evaluation methodology.

2) Visual data generator (_VisualDataGenerator.cs_): a C# script to attach to the camera in a Unity project, to generate VI-SLAM visual input data from the ground truth file in an existing SLAM dataset, in your choice of virtual environment.

3) SLAM sequence execution automation (_sensetime_100.sh_): an example shell script to batch-execute SLAM sequences using ORB-SLAM3.  

4) VI-SLAM trajectory evaluation extension (_trajectory.py_): a Python script that extends an existing trajectory evaluation tool, rpg_trajectory_evaluation (https://github.com/uzh-rpg/rpg_trajectory_evaluation), to provide error data at the sub-trajectory level.

5) Sequence characterization and data analysis (_results_analysis.py_): a Python script to analyze the visual and inertial input data in a SLAM sequence, and correlations with pose error.   

# Instructions

Prerequisites: Unity 2020 or later, open-source SLAM algorithm (e.g., ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3). rpg_trajectory_evaluation toolbox (https://github.com/uzh-rpg/rpg_trajectory_evaluation) and Python required for trajectory evaluation and data analysis steps. 

Tested with Unity 2020.3.14f1 (High Definition Render Pipeline) running on Windows 10, ORB-SLAM3 running on Ubuntu 16.04, with TUM VI and SenseTime VI-SLAM datasets.

We implement our VI-SLAM evaluation methodology using the following directory structure, that supports execution with ORB-SLAM3:

![Virtual-Inertial-SLAM directory structure](https://github.com/Virtual-Inertial-SLAM/Virtual-Inertial-SLAM-Resources/blob/main/directory_structure.png?raw=true)

1) Download the VI-SLAM dataset of your choice in a convenient location (e.g., TUM VI: https://vision.in.tum.de/data/datasets/visual-inertial-dataset, SenseTime: http://www.zjucvg.net/eval-vislam/dataset/). You will need the inertial data and the ground truth pose data for the sequences you wish to use.

2) Replicate the directory structure above in a convenient location - because we run Unity on Windows and run VI-SLAM on Ubuntu, we set up a shared folder and place it there. Our sample folder, '_A1_Name.zip_', contains this directory structure for the SenseTime A1 sequence, which you can download, unzip and copy.

3) Create a Unity project and create your desired virtual environment (see the section 'Realistic Virtual Environments' below for tips on how to do this). Set the scene camera to have the desired field of view that you wish to use. Create a render texture with the dimensions of VI-SLAM input images you wish to use; set the color format to 'R8G8B8A8_UNORM' to generate grayscale camera images. Set the render texture under 'Output>Target Texture' of your camera, and set the W and H values of 'Output>Viewport Rect' to match the image dimensions. Attach the visual data generation script provided in this repository (_VisualDataGenerator.cs_) to the camera game object, and within that script, edit the values under 'User-defined parameters' as required, and save.

4) Run the visual data generation script (press 'Play' in Unity). When script has finished executing, the visual data and the necessary config files will have been generated in the 'InputData' folder, including the .yaml file which specifies camera intrinsics and extrinsics.

5) Execute the VI-SLAM sequences in your open-source SLAM algorithm. In this repository we provide an example shell script to batch-execute 100 trials, from 10 different virtual environment settings, using ORB-SLAM3 (sensetime_100.sh). Ensure the paths to the config (.yaml) file and the inertial data are correct, and the inertial data is formatted correctly for your SLAM algorithm. Note the inclusion of the command _sed 's/\r$//'...._ ; this step is necessary to remove carriage returns from one of the config files generated in Windows.

6) We use the rpg_trajectory_evaluation toolbox as a standalone tool to calculate error metrics from the trajectory estimate files generated by an open-source VI-SLAM algorithm. We extend it to write out a summary of the error metrics calculated (results.csv) as well as the error for each sub-trajectory (e.g., sub_errors_A1_1_2.0.csv). To implement this extension replace 'trajectory.py' in 'rpg_trajectory_evaluation/src/rpg_trajectory_evaluation/' with the '_trajectory.py_' file provided in our repository. Run rpg_trajectory_evaluation and copy the output files to the 'Results' folder for your experiment.

7) We use our provided Python script '_results_analysis.py_' to characterize the visual and inertial data, and calculate correlations with trajectory estimate error. Edit the section 'User-defined parameters' as required (lines 74-79), then execute the script using your Python environment of choice. A summary of results in each environment setting (results_summary.csv) plus various plots will be generated in the 'Analysis' folder for your experiment (see samples below).

# Sample Outputs

Below are samples of the plots generated by our '_results_analysis.py_' script, using SenseTime A1 in a virtual museum environment. As well as generating a summary .csv file, this script outputs various .png files to aid researchers in studying the relationship between VI-SLAM input data properties and pose estimation error. For each trial, we analyze data at the sub-trajectory level, and show:

1) The distribution of sub-trajectory errors (left), and sub-trajectory errors over time, by plotting each sub-trajectory error value in order (right):

![Virtual-Inertial-SLAM error analysis](https://github.com/Virtual-Inertial-SLAM/Virtual-Inertial-SLAM-Resources/blob/main/sample_outputs_error.png?raw=true)

2) How input data characterization metrics change throughout the trajectory, for both visual (e.g., Laplacian, left) and inertial (e.g., gyroscope magnitude, right) data:

![Virtual-Inertial-SLAM error analysis](https://github.com/Virtual-Inertial-SLAM/Virtual-Inertial-SLAM-Resources/blob/main/sample_outputs_metrics.png?raw=true)

3) The Spearman correlation coefficients between characterization metrics and error (left), as well as scatter plots for each metric (e.g., Laplacian, right): 

![Virtual-Inertial-SLAM error analysis](https://github.com/Virtual-Inertial-SLAM/Virtual-Inertial-SLAM-Resources/blob/main/sample_outputs_correlations.png?raw=true)

Note: In these last two plots concerning correlations, the sub-trajectories during which the VI-SLAM algorithm is still initializing are removed from analysis. Here we drop the first 100 sub-trajectories, but this value can be adjusted within the _results_analysis.py_ script.

# Realistic Virtual Environments

In order to create high-quality, realistic environments with accurate light rendering, we recommend using Unity's High Definition Render Pipeline (HDRP). To start creating a HDRP Unity project, follow the instructions on this page: https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@15.0/manual/Getting-started-with-HDRP.html

Within a HDRP project Unity provides a sample scene with a pre-built environment. We used this environment to create an example of Virtual-Inertial-SLAM running in a highly realistic virtual environment (using ORB-SLAM3 and the SenseTime A1 sequence), as you can see below:

![GIF of Virtual-Inertial-SLAM in action](https://github.com/timscargill/Virtual-Inertial-SLAM/blob/main/RealisticVirtualEnvironment_crop.gif?raw=true)

More sample HDRP projects are here: https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@15.0/manual/HDRP-Sample-Projects.html

To start creating your own HDRP environment, try searching for 'HDRP' in the Unity asset store. This will present downloadable models and materials which you can add to your project and use to build the realistic virtual environment of your choice.

# Citation

If you use Virtual-Inertial SLAM in an academic work, please cite: 

```
@inproceedings{Virtual-Inertial-SLAM,
  title={Integrated design of augmented reality spaces using virtual environments},
  author={Scargill, Tim and Chen, Ying and Marzen, Nathan and Gorlatova, Maria},
  booktitle={Proceedings of IEEE ISMAR 2022},
  year={2022}
 }
 ```

# Other projects which use Virtual-Inertial SLAM:



# Acknowledgements 

The authors of this repository are Tim Scargill and Maria Gorlatova. Contact information of the authors:

* Tim Scargill (timothyjames.scargill AT duke.edu)
* Maria Gorlatova (maria.gorlatova AT duke.edu)

This work was supported in part by NSF grants CSR-1903136, CNS-1908051, and CNS-2112562, NSF CAREER Award IIS-2046072, and a Meta Research Award. 
