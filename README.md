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


