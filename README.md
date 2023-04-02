
# TurtleSim Task


## Setup:

- Install ROS noetic on Ubuntu 20.04 from here. Install full-desktop install which automatically installs turtlesim.
- Clone this repo to catkin workspace and build it. Source the workspace after building.

Note: Might need to make python files executable. To do that use : `chmod +x /path_to_file`


## Goals

---

### Goal 1: Control Turtle

The aim of this goal is to move the turtle from random spawning point to a goal by controlling the linear and angular velocity. Since the turtlebot is a 2D bot, only linear velocity in x and angular velocity in z needs to be calculated. The rest of velocities are made zero.

The linear velocity is calculated using Euclidean distance, while the angular velocity is calculated using steering angle which is the slope of the line joining 2 consecutive points.

Considering $~q_x ~\text{and}~ q_y~$ to be current pose of the turtle bot and $q_x^g$ and $q_y^g$ to be the goal position, the Euclidean Distance $d$ is calculated as: 

$$
d = \sqrt(q_x - q_x^g)^2 + (q_y - q_y^g)^2
$$

The steering angle is calculated as 

$$
tan^{-1}(\frac{q_y^g - q_y}{q_x^g - q_x})
$$

Proportional Control for linear velocity is a gain value multiplied by linear velocity calculated from Euclidean Distance. For angular velocity it is gain value multiplied by steering angle - yaw of the turtle bot($\theta$).

This can be verified by launching:

`roslaunch turtlesim_task goal_1.launch x_val:=2 y_val:=3`

Here x_val and y_val specify the goal position. If no value is given, the default is set to (1,1). The video result is found [here](https://iiitaphyd-my.sharepoint.com/:v:/g/personal/swati_dantu_research_iiit_ac_in/EdlIWUEFqexAoYZgx9RajYkBp3evWNGV65JDXk4-4QFchw?e=jSb2H2).

**PID Controller:**

In this implementation, the proportional term is same as the previous implementation. The derivative term is calculated as gain_d*(prev_value - current). The integral term is calculated using gain_i*(current+prev_value), so that the errors add up(integrate) overtime.

This can be verified by launching:

`roslaunch turtlesim_task goal_1_PID.launch x_val:=2 y_val:=7`

Here x_val and y_val specify the goal position. If no value is given, the default is set to (1,1). The video result is found [here](https://iiitaphyd-my.sharepoint.com/:v:/g/personal/swati_dantu_research_iiit_ac_in/EWKSkinFO9BCv8zlAoEyliwBSwJ1-_MOB-3WlCVgzoyWGA?e=8R06S3).


### Goal 2 : Rotate turtle in circle

The goal is to make a turtle move in a circle given velocity(v) and radius(r). Since the velocity is already known, the linear velocity is set to v.  The velocity v can be written as $v=r\omega$ where $\omega$ is the angular velocity. Therefore angular velocity is set as $\frac{v}{\omega}$.

After making a turtle \turtle_spawn is made to move in a circle, another instance of turtle /turtle_PT is spawned that follows /turtle_spawn using the topic /rt_real_pose published every 5 sec. The /turtle_PT is the same as the PID from previous goal. 

This can be verified by launching:

`roslaunch turtlesim_task goal_2.launch radius:=3 velocity:=3`

Here radius and velocity variables are to set desired radius and velocity for the /turtle_spawn to follow. By default they are set to radius = 3 and velocity = 2. The video for the same is found [here](https://iiitaphyd-my.sharepoint.com/:v:/g/personal/swati_dantu_research_iiit_ac_in/EdeEYYW_OkxLikNpvrCILHoBYT_YGVDXlDGChCnYUcMbvg?e=clPehV).

As seen from the video, one the /turtle_PT reaches /turtle_spawn, it will remain on the  circle.

### Goal 3 : Chase turtle

A turtle named /turtle_RT is spawned to make it move it a circle with the default radius 3 and default velocity 2 as seen in the previous goal. After 10 sec, another instance named /turtle_PT is spawned randomly that chases after /turtle_RT. Since the radius is set at 3units, the /turtle_PT stops the chase when it is at 0.3 units distance. The /turtle_PT gets pose from the topic /rt_real_pose.

**To set acceleration limits on \turtle_PT:**

Since turtlesim does not have provision to set acceleration, this has been done through velocity. When the acceleration calculated as $\frac{dv}{dt}$ exceeds acceleration limit, the acceleration is set to maximum and velocity final  $v_f$ is calculated from the equation below. Here $dt$ is set to 1/10 because the communication rate is set to 10Hz. (P.S. This is the first time I am imposing acceleration limits via velocity. Thank you for considering.)

$$
a_{max} = a = \frac{dv}{dt} = \frac{v_f - v_i}{d_f-d_i}
$$

This can be verified by launching:

`roslaunch turtlesim_task goal_3.launch`

By default they are set to radius = 3 and velocity = 2. The video for the same is found [here](https://iiitaphyd-my.sharepoint.com/:v:/g/personal/swati_dantu_research_iiit_ac_in/EfRjDGiYPCJIlUXPYKa-E2QBdI4ak_UMDrBl7Um24_KE1Q?e=mv345P).

As seen from the video, where the /turtle_PT reaches within 0.3 units of /turtle_RT, the chase is stopped. It can also be observed from the velocity plot in the video, because of the acceleration limits, while deceleration, we can see that the plot is clipped in certain areas.

### Goal 4 : Escape turtle (Optional)

A turtle named /turtle_RT is spawned that moves randomly. Another instance /turtle_PT is spawned that can move half as fast as /turtle_PT. If the velocity of PT is more that RT/2, the velocity is set to RT/2. PT has access to the velocity of RT as well as /rt_real_pose.

This can be verified by launching:

`roslaunch turtlesim_task goal_4.launch`

The video for the same is found [here](https://iiitaphyd-my.sharepoint.com/:v:/g/personal/swati_dantu_research_iiit_ac_in/EYVwAC_-g8hKgyu1TJrY5DQBnB9UsY2NOkSl1enRMQNZsg?e=KBfZN4). As seen from the video, where the /turtle_PT reaches within 0.3 units of /turtle_RT, the chase is stopped.

