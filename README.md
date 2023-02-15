# ORB-SLAM3

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:

    @article{ORBSLAM3_2020,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={arXiv preprint arXiv:2007.11898},
      year={2020}
     }

# 2. Prerequisites

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.
s of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are ) are included in the *Thirdparty* folder

## ROS

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.


# 4. ROS

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D
Tested with ROS Melodic and ubuntu 18.04.
1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Stereo-Intertial Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
 rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/new.yaml false
  ```
  
  ```
  roscore
  ```
  
  ```
 rosbag play --pause $BAG_NAME /camera/infra1/camera_rect_raw:=/camera/left/image_raw /camera/infra2/image_rect_raw:=/camera/right/image_raw /camera/accel/sample:=/imu

  ```


​
# 5. Rosbag Joining + Data Generation
Ther Accel, Magnetometer data are on seperate topics, this script will join all the rosbag fragments and join thesse two topics for the final dataset
    
    
    pyhton3 bag_merge.py
       
Change the IN_PATH, OUT_PATH in bag_merge.sh. Will look at all the bags in the given folder.


# 6. Pose Visualisation
This script is for visualising the pose from KeyFrames.txt. In .yaml file use: 
    
    System.SaveAtlasToFile: "map_total"

    python3 pose.py
     
Mention setting of map in the file, filename
500 Features with Segmentation Issues

# 7. ROS Integration
Published Topics


    /orb_slam3/camera_pose, left camera pose in world frame, published at camera rate
    /orb_slam3/tracking_image, processed image from the left camera with key points and status text
    /orb_slam3/tracked_points, all key points contained in the sliding window
    /orb_slam3/all_points, all key points in the map

# 8. Running using Launch File 
Running using Launch files triggers rostopics

In one terminal:
```
roslaunch live_camera_node launch_file
```
In another terminal:
```
roslaunch orb_slam3_ros $file_stereo_inertial.launch
```

#9.Using Live ROS Wrapper:
Run in a seperate terminal
```
python3 merge_topics.py
```
This will join linear,angular velocity and compute orientation passed as IMU type
(can ignore orientation if angular velocity is also ignored)

```
roslaunch orb_slam_3 stereo_d435_inertial.launch

```



