## Mighty Thymio platform

[Thymio](https://www.thymio.org/home-en:home) is a small, inexpensive, mass-produced mobile robot with widespread use in primary and secondary education. In order to make it more versatile and effectively use it in later educational stages, including university levels, we have expanded Thymio's capabilities by adding off-the-shelf hardware and open software components. The resulting robot, that we call **Mighty Thymio** (MyT), provides additional sensing functionalities, increased computing power, networking, and full ROS integration.  We provide a [publication and source code](https://github.com/jeguzzi/mighty-thymio) detailing MyT's architecture and applications in advanced educational activities.

The following are the most important parameters of the (real) MyT:

* base

  - length: 11.0 cm
  - width: 11.18 cm
  - height: 7.7 cm

* wheels

  - radius: 2.2 cm
  - width: 1.5 cm
  - axis (distance between wheels centres): 9.35 cm
  - maximal angular speed: ~7.5 rad/s

* kinematics

  - maximal linear speed: ~ 14 cm/s
  - maximal angular speed: ~ 3 rad/s

* proximity (and ground) sensors

  - maximal range (to detect a white surface): ~ 12 cm
  - field of view: 0.3 rad

* camera

  The camera is mounted on a tilt-able joint (downward pitch is positive) with

  - min pitch: -0.34 rad
  - max pitch: 1.3 rad


> More details regarding the thymio base (e.g. schematics, electronics) can be found [here](https://www.thymio.org/en:thymiospecifications). A special software is needed to visualize the schematics (for example blender).

## Gazebo

[Gazebo](http://gazebosim.org/) is a robot simulator. Simulators are used in robotics for many reasons, one of which is to evaluate algorithms without depending on the real robot or on the specific environment. In many applications it is essential to test robot capabilities, such as error handling, resource management, perception, localization, navigation and grasping.
Some advantages of gazebo include:

* Simulate *realistic* physical interactions (powered by physics engines)
* Design robots or import existing models
* Rapidly test algorithms (if your robot breaks at the first try, no big deal)
* Generate testing indoor or outdoor scenarios (`worlds`)

## ROS + Gazebo

`gazebo_ros_pkgs` is a set of ROS packages that provide the necessary interfaces to simulate a robot in the Gazebo 3D rigid body simulator for robots (for more [details](http://gazebosim.org/tutorials?cat=connect_ros) ). It integrates with ROS using:

* ROS messages
* services
* dynamic reconfigure

## Gazebo model of Mighty Thymio

We have designed a *gazebo* model of the MyT and expose a low level controller for it. Using `gazebo_ros_pkgs` we implemented an interface with ROS. Roughly:

```
       Gazebo                                               ROS

Simulates environment       =====>              State (from Odometry), Sensor measurements
and robot                                       such as proximity, camera, IMU

                            topics,
                            services

Low lever                   <=====              Command velocities (v, omega),
controller                                      camera pitch
```

As we use messages and services to communicate, we can employ all tools from ROS ecosystem.

> Ideally, once we are satisfied with the performance of our modules
> running in simulation we could directly apply them to the real robot.
> In practice, as you will see, some tweaks are often needed.

For detailed information regarding the robot model (geometrical and physical properties) visit [this website](https://github.com/jeguzzi/mighty-thymio/blob/master/client.md#robot-parameters)


### Install Gazebo MyT model and plugins

Install the following ROS packages in your `catkin workspace`:

```
cd <your catkin workspace>/src
git clone https://github.com/jeguzzi/thymioid.git --branch client
git clone https://github.com/jeguzzi/ros-aseba.git --branch client
catkin build
```

Test the installation of the simulated robot by executing:

`roslaunch thymioid_description test_gazebo_bringup.launch name:=thymioXX`

where `thymioXX` is the name you will give to the simulated MyT. This name is used as base name for topics that interact with the MyT.

> By default some options of the simulated MyT are not enabled, e.g. the camera sensor. You can select the desired features of the simulated MyT by enabling such options in the `test_gazebo_bringup.launch` file (`args`).


Refer to [MyT interface repository](https://github.com/jeguzzi/mighty-thymio/blob/master/client.md) repository for more details regarding the MyT software and interface. This repository includes information and code for both real and simulated MyT.

> Note: Be aware that the example in **Teleoperate the robot with a joypad** needs a joypad to work properly.

### Install skeleton ROS package for the simulated MyT

Download `thymio_course_skeleton` [package](https://github.com/romarcg/thymio_course_skeleton) for a first glance at how to interact with simulated MyT.

This package includes:
1. Launch files to bring up gazebo simulator and MyT model.
2. Launch files to visualize MyT topics in `rviz`.
3. A basic skeleton node (python script) to interact with the simulated MyT.
4. Extra resources needed for the homework assignment.

