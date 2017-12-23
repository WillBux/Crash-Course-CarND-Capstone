[//]: # (Image References)
[image1]: ./final-project-ros-graph-v2.png "system graph"

## Project Introduction
In this project, we write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. We have tested our code using simulator provided by Udacity.


## Project Team
Will Buxton    
Badri Hiriyur    
Bharat Bobba     
Stas Olekhnovich    
Atul Aggarwal

## ROS Nodes

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![system graph][image1]


### Waypoint Updater
The purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.


### Traffic Light Dectector & Classifier
This node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic /traffic_waypoint.

We use the vehicle's location and the (x, y) coordinates for traffic light stop lines to find the nearest visible traffic light ahead of the vehicle. For the traffic light classification, we utilized the TensorFlow Object Detection API. Using images from the simulator, and images from the Udacity churchlot track, we trained the MobileNet SSD model available in the TensorFlow pretrained model zoo to derive frozen inference graphs. Our repo contains two frozen inference graphs for the simulator, and real-world testing respectively.

### DBW
Here we implement drive-by-wire node using PID controller to provide appropriate throttle, brake and steering commands.  We use Yaw controller to convert target linear and angular velocity to steering commands.We create 2 PID controllers for throttle/brake and steering respectively. We take difference between target steering (provided by Yaw controller) and current steering angle as error in PID controller for steering. PID controller for throttle/brake takes difference between target velocity and current velocity as error. After calulating next throttle, brake and steering controls, DBW node publishes them to car/simulator node. 


## Simulator performance (simulator)
## Churchlot site performance (simulator)
## Churchlot site performance (Carla)
