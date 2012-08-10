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

You can view the robot in rviz by adding a "Robot Model".


Joystick Controls
-----------------

The joystick controls currently offered by `barc_nao_joy` on a Logitech Rumble Pad are as follows:

 + *Button 9* -- Toggle gamepad control on or off. Toggling control on also turns stiffness on. This must be done before the controller can be used to drive the robot. Toggling off does not turn stiffness off (use *Button 10* instead).


 + *Button 10* -- Send the robot into a stable crouch position and turn off joint stiffness. This is a good way to leave the robot in between runs. To turn stiffness back on use *Button 9*.


 + When holding *Button 8* the following buttons run behaviours that have been previously uploaded to the Nao in Choreographe. They are run based on names and are currently configured as follows: *Button 4* "stand_up" (the standard stand-up from any position move), *Button 1* "sit_down" (the standard sit down from any position move), *Button 3* "say_hello" (the wave), and *Button 1* "wipe_brow".



