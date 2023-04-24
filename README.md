# LiDAR_Tracking_3D

This project track the person in indoor environment using 3D LiDAR. <br> 

##  Abstract

<p> Light Detection and Ranging (LiDAR) technology
is now becoming the main tool in many applications such as
autonomous driving and human robot collaboration. Point-cloud
based 3D object detection is getting popular and widely accepted
in industry and everyday life due to its effectiveness against
cameras in challenging environment. In this paper, we present
a modular approach to detect, track and classify persons using
a 3D LiDAR sensor. It combines multiple principles: a robust
implementation for object segmentation, a classifier with local
geometric descriptors and a tracking solution. Moreover, we
achieve a real time solution in a low-performance machine by
reducing the amount of points to be processed by obtaining and
predicting regions of interest via movement detection and motion
prediction without any previous knowledge of the environment.
Furthermore, our prototype is able to successfully detect and
track persons consistently even in challenging cases due to
limitations on the sensor field of view or extreme pose changes
such as crouching, jumping, and stretching. Lastly, the proposed
solution is tested and evaluated in multiple real 3D LiDAR sensor
recordings taken in indoor environment. The results show great
potential, particularly a high confidence in positive classifications
of human body as compared to state-of-the art approaches. <p>
  
  <b> ROS Package: </b>
  
  ```
cd ~/catkin_ws/src
```
clone the package
  
   ```
git clone https://github.com/baberjunaid/LiDAR_Tracking_3D.git

```
finally build the package  
  
  ```
cd ~/catkin_ws/  
  catkin_make  
  source devel/setup.bash
```
  
To run the launch file
  
  ```
roslaunch LiDAR_Tracking_3D Track_3D.launch input:=/ouster/points

```
  
 where /ouster/points contains the node of point cloud. We have used Ouster lidar. If you are using /velodyne_points then update the value of input accordingly. 

## Description
  
  This project tracks multiple people in an indoor environment using 3D LiDAR (Ouster) in real-time. For a given 3D point cloud, ROI is extracted that reduces the number of point clouds, [ROI Demo](https://youtu.be/pKhH1pguWy8 "ROI Demo"). Later, these ROIS are passed to voxelization and segmentation. The segmented objects later classified into person and on-person. The demo of person tracking and detection can be seen on [Tracking Demo](https://lig-membres.imag.fr/aycard/html//Projects/JuanGomez/JuanGomez.html).   
  
  
  [ROI Demo](https://youtu.be/pKhH1pguWy8 "ROI Demo"), [Tracking Demo](https://lig-membres.imag.fr/aycard/html//Projects/JuanGomez/JuanGomez.html)


