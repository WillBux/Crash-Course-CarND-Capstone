### Project Introduction
In this project, we write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. We have tested our code using simulator provided by Udacity.


### Project Team
Will Buxton
Badri Hiriyur
Bharat Bobba
Stas Olekhnovich
Atul Aggarwal

### ROS Nodes

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

## Waypoint Updater
The purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.


## Traffic Light Dectector & Classifier
This node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic /traffic_waypoint.

We use the vehicle's location and the (x, y) coordinates for traffic light stop lines to find the nearest visible traffic light ahead of the vehicle. After that we train a Deep convolutional Neural Network on camera images to classify the color of the traffic light. Architecture of our Neural Network is as below:-

```
_________________________________________________________________
Layer (type)                 Output Shape              Param #
=================================================================
conv2d_4 (Conv2D)            (None, 300, 400, 16)      448
_________________________________________________________________
leaky_re_lu_4 (LeakyReLU)    (None, 300, 400, 16)      0
_________________________________________________________________
max_pooling2d_4 (MaxPooling2 (None, 100, 133, 16)      0
_________________________________________________________________
dropout_5 (Dropout)          (None, 100, 133, 16)      0
_________________________________________________________________
conv2d_5 (Conv2D)            (None, 100, 133, 32)      4640
_________________________________________________________________
leaky_re_lu_5 (LeakyReLU)    (None, 100, 133, 32)      0
_________________________________________________________________
max_pooling2d_5 (MaxPooling2 (None, 33, 44, 32)        0
_________________________________________________________________
dropout_6 (Dropout)          (None, 33, 44, 32)        0
_________________________________________________________________
conv2d_6 (Conv2D)            (None, 33, 44, 64)        18496
_________________________________________________________________
leaky_re_lu_6 (LeakyReLU)    (None, 33, 44, 64)        0
_________________________________________________________________
max_pooling2d_6 (MaxPooling2 (None, 16, 22, 64)        0
_________________________________________________________________
dropout_7 (Dropout)          (None, 16, 22, 64)        0
_________________________________________________________________
flatten_2 (Flatten)          (None, 22528)             0
_________________________________________________________________
dense_3 (Dense)              (None, 128)               2883712
_________________________________________________________________
dropout_8 (Dropout)          (None, 128)               0
_________________________________________________________________
dense_4 (Dense)              (None, 3)                 387
=================================================================
Total params: 2,907,683
Trainable params: 2,907,683
Non-trainable params: 0

```

## DBW
Here we implement drive-by-wire node using PID controller to provide appropriate throttle, brake and steering commands.  We use Yaw controller to convert target linear and angular velocity to steering commands.We create 2 PID controllers for throttle/brake and steering respectively. We take difference between target steering (provided by Yaw controller) and current steering angle as error in PID controller for steering. PID controller for throttle/brake takes difference between target velocity and current velocity as error. After calulating next throttle, brake and steering controls, DBW node publishes them to car/simulator node.


### Simulator performance (simulator)
### Churchlot site performance (simulator)
### Churchlot site performance (Carla)
