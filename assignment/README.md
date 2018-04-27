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

> In theory, once we are satisfied with the performance of our modules
> running in simulation we could directly apply them to the real robot.
> In practice, as you will see, some tweaks are always needed.

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

> Note: this homework assignment **will be graded**.


**Compulsory tasks** using Gazebo and the simulated MyT:

1. Write an open loop controller that moves the MyT in such a way that it follows an "8" trajectory. Test it in the `empty` world file.

2. Using the `wall` world file write a controller to move the MyT straight ahead; we assume the robot is heading towards a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to.  Write a controller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting the wall), then turns in place in such a way to face the wall as precisely as possible (i.e., the robot's x axis should be orthogonal to the wall).  In order to sense the wall orientation once you are close to it, you should use the proximity sensors.  Feel free to define a convenient distance threshold at which you decide to stop.

3. Using the controller built in *task 2*, once the MyT arrives close to the wall, it should then turn in such a way that it is facing opposite to the wall, then move and stop in such a way that its reference frame (i.e. `base_link` in `tf` tree) is as close as possible to a point that is 2 meters from the wall.  Note that the proximity sensors don't have such a long range, so at some point you'll have to rely on odometry.

**Optional tasks** using the real MyT (to be used in the real world)

4. Test *task 1* on a flat floor. How does the resulting trajectory compare to the trajectory you found in simulation?

5. Test *task 2* against a generic wall-like obstacle (e.g. a wall). How precise is the final angle you reach?
Test your controller using walls of different colors, and also a wall that has a non-uniform color. The most extreme case is the case in which the thymio is facing the wall at a point, and the wall is black on one side of this point and white on the other side.

*Tasks 4 and 5* may require tuning of parameters to improve the performance of the controllers.  We encourage to compare the performance of the controllers with (if needed) and without tuning.


### Submission

> **Note:** students should work in pairs for this assignment; first, enroll in a group with your mate (there is a link for that on iCorsi).  If you are working alone, enroll in a 1-person group.  In any case, you'll submit as a group (a single submission per group is required).

The submission should contain:
* The full code you implemented (package folder), including brief instructions on how to run and test it.
* A brief document stating, for each task:
    * whether your solution implements it;
    * if yes, whether you still have any doubts/questions/puzzling behavior in your solution
    * if no, which problems you encountered
    * any additional comments or notes, including any extra work or experiments you did; feel free to attach screenshots or video links.
* For *task 1*, an `rviz` snapshot of the `odometry visualization` trace.
* For *tasks 4 and 5*, a short animation (e.g. video, GIF) of the real MyT while executing your controller.  Bear in mind the size of the animation file (or include a link to the file on dropbox/wetransfer).

Deadline: **Friday May 4 8h30 Lugano time**
