## ROS

## TL:DR

Here's a collection of useful ROS packages that do the stuff you thought you were going to have to program yourself - at least do the basic tutorials first though.

The real cheat sheet.

 - [Extended Kalman Filter](http://wiki.ros.org/robot_pose_ekf) (sensor fusion, e.g. IMU + Odometry + ...)
 - [Navigation](http://wiki.ros.org/navigation) (...)
 - [Hokuyo](http://wiki.ros.org/urg_node) (Laser scanner driver)
 - [ROS Control](http://wiki.ros.org/ros_control) (PID controllers that work both on hardware and simulation. Documentation is questionable)
 - [Controllers](http://wiki.ros.org/ros_controllers) (Controllers available in ROS Control)
 - [rviz](http://wiki.ros.org/rviz) (visualising data)
 - [MoveIt!](https://moveit.ros.org/) (robot arms etc.)
 - [Camera Callibration](http://wiki.ros.org/camera_calibration) (...)

### Overview

Robot Operating System (ROS) is a software framework for robot software development. It has considerable support with many useful packages readily available online. It can be run on Unix-based platforms like Ubuntu on devices from a Raspberry Pi to a normal PC. It enables visualisation of the nodes and processes, which is very helpful when designing and debugging the robot software. Simulation of the robot's operation and actions is also available through ROS (with Gazebo). Most ROS programs are programmed with C++ and/or Python but theoretically the framework is language-independent. For more details, documentation and tutorials check the [ROS Wiki](http://wiki.ros.org/).

The main advantage is to allow separation of the different components. This means that if a bug in e.g. the IMU processing node causes it to crash the rest of the robot can continue until that node restarts. Isolating parts of the robot also makes development and debugging much easier.

### Basic Tutorials

#### Installation

Although it's possible to directly install ROS on Windows, ROS is designed to run in Linux. It's recommended to install ROS in Ubuntu using the tutorial [here](http://wiki.ros.org/melodic/Installation/Ubuntu), you can download the latest Ubuntu for desktop [here](https://ubuntu.com/download/desktop). Currently we use the Melodic distribution of ROS as it is the most recent LTS. If you really want to stick with Windows there is an installation method [here](http://wiki.ros.org/Installation/Windows).

#### ROS basics

Conveniently, ROS wiki has provided a [beginner tutorial](http://wiki.ros.org/ROS/Tutorials#Beginner_Level) which covers most basic ROS concepts and the use of Services, roslaunch, ROS message, publisher and subscriber and other common ROS features. Please navigate through the tutorial and feel free to learn more yourself from the [intermediate tutorial](http://wiki.ros.org/ROS/Tutorials#Intermediate_Level) and other sources.

There is also a useful introductary online course for ROS in Chinese [here](https://www.icourse163.org/course/ISCAS-1002580008), good luck with that;)

### TF

TF is the ROS transformations library. The most up to date version is [tf2](http://wiki.ros.org/tf2), but it's backwards compatible with the original tf.

TF allows you to easily convert between coordinate frames in your robot. For example, a point you detected from a camera is great, but knowing where that is relative to the grabber is vital to be able to grab it. TF handles all these complex transformations for you.

The [tutorials](http://wiki.ros.org/tf2/Tutorials) are super handy, particularly #2, #3 and #5. Play with them!

Typical C++ code to get transform a point:
```cpp
// Standard ROS include
#include <ros/ros.h>
// The tf2 transform listener
#include <tf2_ros/transform_listener.h>
// The transform class
#include <geometry_msgs/TransformStamped.h>
// Point compatible with tf
#include <geometry_msgs/Point.h>

// Eigen for fancy maths we might later end up doing...
#include <Eigen/Geometry>

// TF2 setup stuff
// The listener will catch messages and store them in the buffer for ~10s
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener tf_listener(tf_buffer);

void some_callback(some_msg_package::some_msg::ConstPtr msg) {
    // ... Acquire data from somewhere ...
    Eigen::Vector3f point_in_frame_a;

    // Get the tf transform from A to B
    geometry_msgs::TransformStamped a_to_b_tf;
    auto now = ros::Time::now();
    float secs = now.secs;
    float nsecs = now.nsecs;
    try {
        tf_buffer.waitForTransform("b", "a", ros::Time(secs, nsecs), 
                                   ros::Duration(0.05));
        a_to_b_tf = tf_buffer.lookupTransform("b", "a", 
                                              ros::Time(secs, nsecs));
    }
    catch(tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Got an error: " << ex.what());
        // Return as we failed for some reason
        // Likely reason is that we didn't wait long enough so the 
        // transform hasn't been published yet, or we were asking for
        // a transform a long time in the past
        return;
    }

    // TF transforms don't work on Eigen data types, so you need to convert
    // from one to the other. In this case we'll convert from Eigen
    // to TF, but sometimes it's easier to do it the otherway.

    // Note that you could use geometry_msgs::Vector3, but it won't get
    // translated by tf as it only represents a direction.
    // It would still be rotated though.
    geometry_msgs::Point p;
    p.x = point_in_frame_a.x();
    p.y = point_in_frame_a.y();
    p.z = point_in_frame_a.z();

    p = a_to_b_tf * p;

    Eigen::Vector3f point_in_frame_b(p.x, p.y, p.z);

    // Continue function using point in new co-ordinate frame...
}
```

### Rviz

[Rviz](http://wiki.ros.org/rviz) is a great tool in ROS. It allows quick visualisation of what's going on in your ROS system, from looking at laser scan data to viewing the TF graph of your robot.

To boot it up simply run `rosrun rviz rviz`. If you want to use a particular config file (to remember which topics to listen to for example) use `rosrun rviz rviz -d /path/to/rviz_config.rviz`.

The tutorials aren't great unfortunately. [This](https://www.youtube.com/watch?v=KYIOlTgi-J4) is a decent video that's only 5 minutes long. It starts off with a little bit of URDF and roslaunch too.

### URDF

URDF stands for Unified Robot Description Format. It provides a way to describe the positions and joints of your robot, such as torque limits, velocity limits, degrees of freedom and position limits.

The URDF [tutorials](http://wiki.ros.org/urdf/Tutorials) are fantastic, quickly showing you how to build a simple robot, including using the robot with Gazebo.

### Robot State Publisher and Joint State Publisher

Robot State Publisher is vital for your robot. It converts your URDF into a TF graph and calculates the forward kinematics once the robot begins to move around. [Tutorial](http://wiki.ros.org/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot).

[Joint State Publisher](http://wiki.ros.org/joint_state_publisher) is useful early on for checking that all joints are set up correctly as it allows you to move the joints by hand via a GUI.

Example launch file:
```xml
<launch>
    <!-- Load robot URDF description -->
    <param name="robot_description" 
           command="cat $(find my_robot_description)/urdf/my_robot.urdf"/>

    <!-- robot state publisher -->
    <node name="robot_state_publisher" pkg="robot_state_publisher"
          type="robot_state_publisher" respawn="false" output="screen"/>

    <!-- Launch hardware stuff -->
    <include file="$(find my_robot_hardware)/launch/hardware.launch"/>

    <!-- Launch navigation stuff -->
    <include file="$(find my_robot_nav)/launch/navigation.launch"/>
    
    <!-- etc.. -->
</launch>
```

### Rosserial

[Rosserial](http://wiki.ros.org/rosserial) is really just a wrapper around Serial communication, but it's pretty handy. It lets you communicate with a microcontroller as if it's just another ROS node on your machine. There's a dedicated package for Arduino and mBed.

The tutorials are on the wiki above.

### ROS Control

Oooh boy. This guy is such a pain, coz the [documentation](http://wiki.ros.org/ros_control) is atrocious. The idea is that a suite of dedicated ROS controllers can perform everything _apart_ from actually sending a PWM signal to your motors. This means that _as much as possible_ your simulated robot and real robot are identical.

There's a great tutorial on how to set up your hardware [here](https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot) and Gazebo has a nifty tutorial on how to link them together [here](http://gazebosim.org/tutorials/?tut=ros_control).

All the controllers are essentially PID controllers, so you need to tune them. This can be done by following the tutorial [here](http://gazebosim.org/tutorials/?tut=ros_control#UseRQTToSendCommands) (uses Gazebo).

Here's a brief run-down of the commonly used controllers:
 - **effort_control**/
     - **JointEffortController**: Tries to control the joint effort by sending it a target effort
     - **JointVelocityController**: Tries to control the joint velocity by sending it a target effort
     - **JointPositionController**: Tries to control the joint position by sending it a target effort
 - **velocity_control**/
     - **JointEffortController**: Tries to control the joint effort by sending it a target velocity
     - **JointVelocityController**: Tries to control the joint velocity by sending it a target velocity
     - **JointPositionController**: Tries to control the joint position by sending it a target velocity
 - **position_control**/
     - **JointEffortController**: Tries to control the joint effort by sending it a target position
     - **JointVelocityController**: Tries to control the joint velocity by sending it a target position
     - **JointPositionController**: Tries to control the joint position by sending it a target position
 - **diff_drive_controller**: Tries to control a differential drive robot by sending the robot wheels velocity commands. Also calculates the robot's odometry.

The idea is that these will output a position, velocity or effort that you then pass to your MCU. That then executes a control loop to control the motor with the ros_control command as the input. E.g. using effort control the control loop could simply be `set PWM to command_effort/max_command_effort`, while a velocity command could feed into another PID loop in the MCU.

### Navigation Stack

The ROS [navigation stack](http://wiki.ros.org/navigation) contains modules to get your robot moving around autonomously. The general idea is to have AMCL estimate the location of the robot in a map from e.g. laser scan data, odometry estimate of how the robot has moved from where it started, sensor information feeding into local and global costmaps, a map server containing the full map of the environment, local and global planners generating paths, an input goal and finally an output velocity for the robot to follow.

Complicated, right?

Let's break it down a bit.

#### AMCL

[AMCL](http://wiki.ros.org/amcl) is a package designed to update the estimation of where your robot is in the map. It does this by publishing a transform from the odometry frame to the map frame. This represents estimations of the error in odometry calculations.

AMCL relies on having a good estimation of where the robot is to begin with. It is optional - to not use it, simply publish a static identity transform from the odometry frame to the map frame.

#### Odometry

Odometry is the measurement of robot motion from the wheels. How this is done will depend on the robot, typically via [differential drive](https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot) or [mecanum drive](http://robotsforroboticists.com/drive-kinematics/) kinematic equations.

Odometry can also be visual, for example measuring the motion of the robot relative to a known part of an image from a camera, or localising via AprilTags etc.
##### Transform between cmd_vel and actual wheel velocity (for holonomic robots)

There is an [article](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf) on transform between cmd_vel and actual wheel velocity (for holonomic robots) 

##### EKF

An Extended Kalman Filter can be used to get an improved odometry estimation, by fusing data from any combination of odometry, visual odometry or IMUs. This attempts to estimate the position of the robot based on the previous positions of the robot and the current known velocity. ROS has an implementation of EKF [here](http://wiki.ros.org/robot_pose_ekf), which has a great diagram of the kind of improvements you can expect to get.

EKF typically will then output a TF called `/odom_combined` and/or on a topic called `/odom_combined`. This can then be fed into anything that normally received odometry.

#### Map Server

The ROS [map_server](http://wiki.ros.org/map_server) contains a map of the environment. In most of our competitions we know the map in advance, so it's simply a PNG image loaded on boot up. 

Its also possible to use this with SLAM, there's probably a tutorial somewhere (see [gmapping](http://wiki.ros.org/gmapping)).

#### Global and Local planners

These bad boys do the actual path planning. The global one generates the full path to be followed, while the local one attempts to follow small segments. These interface with the respective costmaps when deciding which route to take. 

There are a couple of different types [here](https://github.com/ros-planning/navigation), for local the *base_local_planner* and *dwa_local_planner* for global the *global_planner* and *carrot_planner*.

base_local_planner and global_planner are both pretty good tbh.

#### Cost map

You don't really need to interface with these, they just kinda work. I think they also listen to laser scan or other sensor data and update themselves, but I'm not super sure. The general idea though is that they start from the goal point and then increment the next empty grid with the cost and then find the path with the lowest cost.

You might be able to interface with this to weigh certain areas of the map (e.g. enemy areas) as higher or lower than others, but I've never tried.

#### Input Goals

These can be fed directly into the [move_base](http://wiki.ros.org/move_base) topic `move_base_simple/goal`, but it is preferable to use the SimpleActionClient as this provides [extra functionality](https://docs.ros.org/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionClient.html). move_base is already configured with an implementation of the SimpleActionServer which processes goals. The SimpleActionClient is the interface used to send and manage the goals which the SimpleActionServer processes (see [actionlib](http://wiki.ros.org/actionlib) documetation for more information). A simple tutorial for setting goals can be found [here](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals).

Example code from tutorial:
```cpp
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
```

The SimpleActionClient provides a simple interface to set/cancel goals, check progress, e.t.c. These tools are particularly useful when implementing a broader stategy for a robot/competition e.g. what do to when you haven't made progress towards reaching your current goal within a certain amount of time.

#### Output Velocity

Finally, the navigation stack will output on the `/cmd_vel` topic. This tells the robot which direction to go and at what speed. It is up to the robot to then convert this to individual wheel velocities, interface with ros_control and send commands directly to the hardware.

#### Full Navigation Tutorial

There's a decent navigation setup tutorial [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup). Once you've done that it's up to you to find out which parameters to tune though.

### Gazebo Simulations

Gazebo is a fantastic simulation environment. The tutorials for interfacing with ROS begin [here](http://gazebosim.org/tutorials?tut=ros_overview).

If you've set everything else up correctly all you need to do are the following tutorials:
 - [gazebo roslaunch](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
 - [gazebo urdf](http://gazebosim.org/tutorials/?tut=ros_urdf)
 - [gazebo ros_control](http://gazebosim.org/tutorials/?tut=ros_control)
