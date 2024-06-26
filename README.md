# TurtleBot3 Trajectory Recorder and Visualizer

This README provides instructions for setting up and using the TurtleBot3 Trajectory Recorder and Visualizer package. This package allows you to record the trajectory of a TurtleBot3 robot as it moves through its environment and visualize it in ROS2 Humble. 

**The recorded trajectory data is stored in CSV format with a specific filename and duration.**


## Installation

Before using this package, ensure you have the necessary dependencies and packages installed. This package assumes you have already set up ROS2 Humble and have a working installation of TurtleBot3.

**Clone the repository into your ROS2 workspace and build it:**

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/Shashwat1524/TurtleBot3-Trajectory-Recorder-and-Visualizer.git .
cd ..
colcon build
```

## Usage

**Launching TurtleBot3 in Gazebo**

To launch TurtleBot3 in Gazebo, run the following command:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

<br>
<img src="images/1.png" alt="Alt text" width="500">

**Recording Trajectory**

To record the trajectory of the TurtleBot3 robot, run the trajectory node:

```bash
ros2 run my_robot_trajectory_publisher trajectory_node
```

The trajectory node collects the path followed by the robot and publishes the trajectory data as a marker array for real time visualisation.


The package also provides a ROS2 service that allows users to save trajectory data to a CSV file. Users can specify the duration(in seconds) for which they want to save the trajectory data.

**To save trajectory data to a file, use the following command:**

```bash
ros2 service call /trajectory_service_test anscer_interface/srv/TrajectoryPath "{file_name: 'name_of_your_file.csv', duration: 10}"
```

Replace 'name_of_your_file.csv' with the desired file name and 10 with the duration for which you want to save the trajectory data.
Visualizing Trajectory

**After saving the trajectory data, you can visualize it by running the trajectory reader node:**

```bash
ros2 run my_robot_trajectory_publisher trajectory_reader_node --ros-args -p file_name:=name_of_your_file.csv
```

The trajectory reader node reads the saved trajectory file, transforms the trajectory data to the odom frame, and publishes it for visualization.

```bash
rviz2 rviz2
```
Select the Marker Array option and subscribe to trajectory_reader topic.
<br>
<img src="images/2.png" alt="Alt text" width="500">

## Custom Service Interface
The custom service interface **anscer_interface** allows users to request trajectory data saving and provides a response indicating the success of the operation. The request includes the file name and duration, while the response indicates whether the trajectory data was saved successfully.
