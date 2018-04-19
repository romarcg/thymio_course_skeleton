## Mighty Thymio platform

[Thymio](https://www.thymio.org/home-en:home) is a small, inexpensive, mass-produced mobile robot with widespread use in primary and secondary education. In order to make it more versatile and effectively use it in later educational stages, including university levels, we have expanded Thymio's capabilities by adding off-the-shelf hardware and open software components. The resulting robot, that we call **Mighty Thymio** (MyT), provides additional sensing functionalities, increased computing power, networking, and full ROS integration. We provide a [publication and source code](https://github.com/jeguzzi/mighty-thymio) detailing MyT's architecture and applications in advance educational activities.

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


## Gazebo

[Gazebo](http://gazebosim.org/) provides robot simulation, as many other simulators, was designed to evaluate algorithms for robots without depending on the real robot. In many applications it is essential to test robot modules, such as error handling, resource management, perception, localization, navigation and grasping.
Some advantages of gazebo include:

* Test *realistic* interactions (powered by physics engines)
* Design robots or import existing models
* Rapidly test algorithms (without fearing breaking something the first try)
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

Low lever                   <=====              Command velocities (v, w),
controller                                      camera pitch
```

As we use messages and services to communicate, we can employ all tools from ROS ecosystem.

> In theory, once we are satisfied with the performance of our modules
> running in simulation we could directly apply them to the real robot.
> In practice, as you will see, tweaks need to be done.

For detailed information regarding the robot model (geometrical and physical properties) visit [this website](https://github.com/jeguzzi/mighty-thymio/blob/master/client.md#robot-parameters)


### Install Gazebo MyT model and plugins

Refer to [MyT interface repository](https://github.com/jeguzzi/mighty-thymio/blob/master/client.md)

Be aware that the example for `Teleoperate the robot with a joypad` needs a joypad to work properly.

### Install Utility ROS package

Download `thymio_course_skeleton` [package](https://github.com/romarcg/thymio_course_skeleton) for a first glance at how to interact with simulated MyT.

This package includes:
1. Launch files to bring up gazebo simulator and MyT model.
2. Launch files to visualize MyT topics in `rviz`.
3. A basic skeleton python script to interact with the simulated MyT.
4. Extra resources needed for the homework assignment.

## Homework assignment

> Disclaimer: This homework assignment **will be graded**.


(**compulsory**) With Gazebo and the simulated MyT:

1. Write an open loop controller that moves the MyT in such a way that it follows an "8" trajectory. Use the `empty` world file.

2. Using the `wall` world file write a controller to move the MyT towards a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to.  Write a contrexcersieoller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting the wall), then turns in such a way to face the wall.

3. Using the controller built in *task 2*, once the MyT arrives close to the wall, it  now turns in such a way that it is facing opposite to the wall, then moves in such a way that its reference frame (i.e. `base_link` in `tf` tree) is as close as possible to 2 meters from the wall.

For *task 1* we request (in addition to the code) an `rviz` snapshot of the `odometry visualization` trace.

*Tasks 2 and 3* require the use of the proximity sensors. Using this information, feel free to define a *close to the wall* distance threshold.

(**Optional**) With the real world and the real MyT:

4. Test *task 1* on a real flat surface. How does the resulting trajectory compare to the one from simulation?

5. Test *task 2* against a generic wall-like obstacle (e.g. a wall). How *close to the wall* to the wall does the real MyT get?
In addition, test your controller with the real MyT facing orthogonally to the midpoint of a *B&W wall-like* (half black,  half white).

*Tasks 4 and 5* could require tuning of parameters to improve the performance of the controllers. We encourage to compare the performance of the controllers with (if needed) and without tuning. In addition to the code, we request a short animation (e.g. video, GIF) of the real MyT while executing *tasks 4 and 5*. Bare in mind the size of the animation file.

### Submission

> **Note:** You can do this homework assignment in pairs.

Please submit the archive of the package folder to `iCorsi` by **deadline-to-be-defined** , named in the following way: `firstname1.lastname1-firstname2.lastname2.zip`.
