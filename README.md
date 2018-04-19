## Skeleton files to bring up gazebo and the Mighty Thymio model

Add and install the `thymio_course_skeleton` package to your `your_catkin_workspace`:

```
cd <your_catkin_workspace>/src
git clone https://github.com/romarcg/thymio_course_skeleton.git
catkin build
```

### Refer to the launch files in `launch` folder to:

1. Bring up gazebo and the MyT's model. You must name your simulated MyT with the `name` parameter, e.g. `thymio10`. In addition you must indicate a `world` to simulate:

```
roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=empty
```

Examples of world definitions are stored in `.world` files in `launch/worlds/`. Two worlds are included, one `empty` with a large plane and another with a large plane and a `wall`.

> If you want to modify the MyT's `spawing` pose, explore `thymio_gazebo_bringup.launch` file.


2. Visualize a full set of topics exposed by the simulated low-level MyT controller:

```
roslaunch thymio_course_skeleton thymio_rviz.launch name:=thymio10
```

> All topic names from the simulated MyT have as prefix
> the chosen name, e.g. `/thymio10/odom`


### Refer to the skeleton rospy node to:

Perform basic interfacing tasks:
* A simple controller that sets constant velocities by publishing `cmd_vel` messages.
* A subscriber that receives odometry information from simulation.

`basic_move.py` indefinitely performs the above tasks on a simulated MyT. It requires the MyT's name to interface with the correct topics.

```
rosrun thymio_course_skeleton basic_move.py thymio10
```
