Nao
===

Running the Nao using this stack assumes you have a machine running ROS on the same network as the Nao. 

Install
-------

To install the ROS parts, make sure the directory containing this stack is in your ROS_PACKAGE_PATH then do `rosmake nao_demo`. 

To run the components, you also need the NAOqi SDK installed on the same machine. Download the appropriate verison of the "NAOqi C++ SDK" from the [Aldebaran Robotics user area](http://users.aldebaran-robotics.com/index.php?option=com_content&view=article&id=5&Itemid=17) (ask Nick for the login details), then extract it somewhere on your machine (I usually place it under `/opt/naoqi`). To ensure that the ROS components can find the NAOqi SDK, you need to add the lib directory to your PYTHONPATH, and also define the NAOQI_LIBRARY_PATH variable which we'll use later when running components e.g.

```bash
# Where the NAOqi SDK is installed
export NAOQI_HOME=/opt/naoqi/naoqi-sdk-1.12.5-linux64
export NAOQI_LIBS=$NAOQI_HOME/lib

# Add NAOqi to your python path
export PYTHONPATH=$PYTHONPATH:$NAOQI_LIBS
# Define the library path for running ROS Nao components
export NAOQI_LIBRARY_PATH=$NAOQI_LIBS:$LD_LIBRARY_PATH

```
Networking
----------

Press the Nao's chest button once to hear it report its IP address. If you are running the naonet wireless network, this should be 192.168.0.6. 

To ensure the ROS components can find the Nao, set the environment variable NAO_IP to its address, e.g.

```bash
export NAO_IP=192.168.0.6
```
Launching
---------

To launch the components that provide access to Nao functionality, run the following command

```bash
 LD_LIBRARY_PATH=$NAOQI_LIBRARY_PATH roslaunch nao_components nao.launch
```
The override of the LD_LIBRARY_PATH is currently necessary for NAOqi. **Do not** set this in your .bashrc as it contains libraries that could screw up your general settings.

To launch the teleoperation demo, edit `nao_demo/launch/nao_teleop_demo.launch` to set your joystick device correctly, then run

```
roslaunch nao_demo nao_teleop_demo.launch
```

Visualisation
-------------

See http://www.ros.org/wiki/nao/Tutorials/Getting-Started from which the following steps are taken.

First you need to launch the nao_state_publisher node and get it to load the URDF model of the Nao robot

```
roslaunch nao_description nao_state_publisher.launch
```

Then start rviz

```
rosrun rviz rviz
```

In the "Displays" window, change "Fixed frame" to "/base_link"

The "Target Frame" should be "&lt;Fixed Frame&gt;"

Global Status should change to "OK". 
If it's red and "error" then that probably means that the topic /joint_states is not being updated.

Click on the "Add" button and add a grid

Click on the "Add" button and add a RobotModel 

If everything is okay, you will see a robot model made of cylinders.

Joystick Controls
-----------------

The joystick controls currently offered by `barc_nao_joy` on a Logitech Rumble Pad are as follows:

 + **Button 9** -- Toggle gamepad control on or off. Toggling control on also turns stiffness on. This must be done before the controller can be used to drive the robot. Toggling off does not turn stiffness off (use *Button 10* instead).


 + **Button 10** -- Send the robot into a stable crouch position and turn off joint stiffness. This is a good way to leave the robot in between runs. To turn stiffness back on use **Button 9**.

 + When holding **Button 5** the following buttons put the robot into a predined pose. Joint trajectories are interpolated from the robot's current position, but no collision checking etc. is done (so the robot may crash or fall over if you're not in an appropriate start position). The buttons work as follows: **Button 4** raise one arm up, **Button 1** reach both arms out in front of the robot.

 + When holding **Button 6** the analogue sticks control the position of the robot's head. The left stick controls rotation (pan) and the right stick controls translation (tilt).

 + When holding **Button 7** the following buttons alter the robot's LEDs: **Button 4** red, **Button 1** violet, **Button 3** yellow, and **Button 1** green.


 + When holding **Button 8** the following buttons run behaviours that have been previously uploaded to the Nao in Choreographe. They are run based on names and are currently configured as follows: **Button 4** "stand_up" (the standard stand-up from any position move), **Button 2** "sit_down" (the standard sit down from any position move), **Button 3** "say_hello" (the wave), and **Button 1** "wipe_brow".

 + With no buttons held, **Button 1** puts the robot into a standing init pose, making it ready for anything. This is another joint interpolation, so it should already be done from a compatible position (e.g another standing pose or crouching, not **not** sitting).

 + When none of these buttons are held the analogue sticks control the robot's walk. The left stick controls rotation and the right stick controls translation.

