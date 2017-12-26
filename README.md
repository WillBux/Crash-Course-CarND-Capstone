[//]: # (Image References)
[image1]: ./final-project-ros-graph-v2.png "system graph"

## Project Introduction
In this project, we write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. We have tested our code using simulator provided by Udacity.


## Project Team
- Atul Aggarwal (atul.unicode@gmail.com)
- Bharat Bobba (bharatbobba@gmail.com)
- Will Buxton (will.buxton88@gmail.com)
- Badri Hiriyur (bhiriyur@gmail.com)
- Stas Olekhnovich (stas.olechnovich@gmail.com)

## ROS Nodes

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![system graph][image1]


### Waypoint Updater
The purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. This node subscribes to the following messages:

- /base_waypoints (Lane)
- /current_pose (PoseStamped)
- /traffic_waypoint (Int32)

This node publishes the following message:

- /final_waypoints (Lane)

The /final_waypoints message is built from the base waypoints with the first one being the closest to the /current_pose and thereafter in increasing order. The target velocity for the track is set from the base waypoints that are loaded in (through the `velocity` parameter). This node uses a vector of target velocities for all waypoints. This target velocity is first set to the maximum track velocity for all waypoints by default. Then the velocity is updated if the /traffic_waypoint callback is called. In this case, the velocity is set to zero at the reported stop line (the Int32 message in /traffic_waypoint) and then ramped up in a linear fashion as the distance increases behind the stop line. The velocity ahead of the stop line is set to zero.

### Traffic Light Dectector & Classifier
This node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic /traffic_waypoint.

We use the vehicle's location and the (x, y) coordinates for traffic light stop lines to find the nearest visible traffic light ahead of the vehicle. For the traffic light classification, we utilized the TensorFlow Object Detection API. Using images from the simulator, and images from the Udacity churchlot track, we trained the MobileNet SSD model available in the TensorFlow pretrained model zoo to derive frozen inference graphs. Our repo contains two frozen inference graphs for the simulator, and real-world testing respectively.

### DBW
Here we implement drive-by-wire node using PID controller to provide appropriate throttle, brake and steering commands.  We use Yaw controller to convert target linear and angular velocity to steering commands.We create 2 PID controllers for throttle/brake and steering respectively. We take difference between target steering (provided by Yaw controller) and current steering angle as error in PID controller for steering. PID controller for throttle/brake takes difference between target velocity and current velocity as error. After calulating next throttle, brake and steering controls, DBW node publishes them to car/simulator node. 


## Simulator performance (simulator)
## Churchlot site performance (simulator)
## Churchlot site performance (Carla)
