---
layout: page
title: Getting Started
category: Tutorials
---

## Introduction

We assume that:

1. the stack of tasks has been installed using the installation instruction provided [there](download);
2. you understood the [dynamic graph mechanism](a_dynamic_graph).

In this section are detailed some examples to test the stack of tasks framework on the humanoid robot TALOS.

To visualize the robot, you need to have ROS and the following packages for ROS Kinetic:
```bash
sudo apt-get install ros-kinetic-twist-mux ros-kinetic-joy-teleop ros-kinetic-moveit-ros-move-group ros-kinetic-humanoid-nav-msgs ros-kinetic-play-motion ros-kinetic-ompl ros-kinetic-moveit-planners-ompl ros-kinetic-moveit-simple-controller-manager
```

You also need the following robotpkg binaries:
```bash
sudo apt-get install robotpkg-talos-dev
```

### Setting up your environment

#### Quick start
Copy the bash file [setup-opt-testrobotpkgargs.sh](../shscripts/setup-opt-testrobotpkgarg.sh) in the following directory:
```bash
$HOME/bin
```
The following assume a clean environment where only
```bash
/opt/ros/kinetic/setup.bash 
```
has been called.
You can then either source the file when you want to perform simulation, or temporary modifies your .bashrc file by adding:
```bash
source /home/user/bin/setup-opt-testrobotpkgarg.sh /opt/openrobots 1
```
#### Detailed explanations

The script **setup-opt-testrobotpkgarg.sh** simply set a number of important environment variables given one or two arguments.
The first argument, used in both cases,
is the directory where the robotpkg software is installed.
If present the second argument imposes that these variables are the first ones to be considered in the current terminal.

It is necessary for TALOS because some classical ROS packages must have a lower priority.

```bash
if [ $# -eq 2 ];
then
  export ROBOTPKG_BASE=$1
  export PATH=$ROBOTPKG_BASE/sbin:$ROBOTPKG_BASE/bin:$PATH
  export LD_LIBRARY_PATH=$ROBOTPKG_BASE/lib:$ROBOTPKG_BASE/lib/plugin:$ROBOTPKG_BASE/lib64:$LD_LIBRARY_PATH
  export PYTHONPATH=$ROBOTPKG_BASE/lib/python2.7/site-packages:$ROBOTPKG_BASE/lib/python2.7/dist-packages:$PYTHONPATH
  export PKG_CONFIG_PATH=$ROBOTPKG_BASE/lib/pkgconfig/:$PKG_CONFIG_PATH
  export ROS_PACKAGE_PATH=$ROBOTPKG_BASE/share:$ROBOTPKG_BASE/stacks:$ROS_PACKAGE_PATH
  export CMAKE_PREFIX_PATH=$ROBOTPKG_BASE:$CMAKE_PREFIX_PATH
else
  export ROBOTPKG_BASE=$1
  export PATH=$PATH:$ROBOTPKG_BASE/sbin:$ROBOTPKG_BASE/bin
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROBOTPKG_BASE/lib:$ROBOTPKG_BASE/lib/plugin:$ROBOTPKG_BASE/lib64
  export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python2.7/site-packages:$ROBOTPKG_BASE/lib/python2.7/dist-packages
  export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG_BASE/lib/pkgconfig/
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROBOTPKG_BASE/share:$ROBOTPKG_BASE/stacks
  export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROBOTPKG_BASE
fi
```

## Whole body motion

### Introduction

The following is a quick introduction to a control structure which is implementing a hierarchical inverse kinematics.

### Start the simulation

```bash
roslaunch talos_gazebo talos_gazebo.launch
```

WARNING: The first time you are launching this command it might take some time because gazebo is downloading several models from Internet.

### Start the SoT-ROS interface for TALOS in simulation (Gazebo) 

```bash
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo.launch
```

### Start the motion of the robot
```bash
cd /opt/openrobots/share/sot-talos/tests/
python test.py
```

### Detailed explanation of test.py

This script is run by a python interpreter outside the real-time control loop of the robot.

```python
#!/usr/bin/python
import sys
import rospy
```
The first lines are simply import system modules and the ros python interface.

```python
from std_srvs.srv import *
```
It is used to test if the services provided by the SoT-ROS interface  are available.

```python
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *
```
Import the helper objects to access services provided by the SoT-ROS interface.

```python
def launchScript(code,title,description = ""):
    raw_input(title+':   '+description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line != '' and line[0] != '#':
            print line
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print answer
    rospy.loginfo("...done with "+title)
```
This script is loading a python file which will be send to the embedded python interpreter of the SoT.
It is waiting the user to hit the enter key after display the python file to be loaded.

```python
# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")
```
This part of the script waits that the services provided by SoT-ROS interface become available:
<ul>
<li> <b>run_command which</b> is the service to send python commands.</li>
<li> <b>start_dynamic_graph</b> is the service to start the control of the robot.</li>
</ul>


```python
    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)
```
Two helper objects are created to interact with the services.

```python
    initCode = open( "appli.py", "r").read().split("\n")
```
The file **apply.py** explained below is loaded in the variable initCode.
It basically the control graph that will be applied to the robot.

```python
    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
```
The last line is sending the script appli.py to the interpreter on the robot.

```python
    raw_input("Wait before starting the dynamic graph")
```
This line prints the string and waits for the user to hit enter

```python
    runCommandStartDynamicGraph()
```
The last line starts to apply the control law to the robot and evaluate the whole SoT control graph.

```python
    raw_input("Wait before moving the hand")
```
This line prints the string and waits for the user to hit enter

```python
    runCommandClient("target = (0.5,-0.2,1.0)")
    runCommandClient("gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))")
    runCommandClient("sot.push(taskRH.task.name)")
```
The first **runCommandClient** specifies a target position in (X,Y,Z) coordinates.
The second **runCommandClient** specifies the gains to apply to the task **taskRH** and the axis to control.
Here '111' means that all axis are controlled.
The last runCommandClient push the task **taskRH** in the solver.

```
except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)
```
The last two lines deals with exception which might raise during the process.

### Detailed explanations of appli.py

```python
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye
```
The first line import object collecting objects to realize generic kinematic tasks.

```python
taskRH    = MetaTaskKine6d('rh',robot.dynamic,'rh',robot.OperationalPointsMap['right-wrist'])
handMgrip = eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
```
The first line create a task **rh** at the operational point 'right-wrist'.
The second line creates a homogeneous matrix  **handMgrip** . The rotational part is set to identity and the translation part
is set to **(0.1,0.0,0.0)**.
The third line set a modification of the operational point. It is such that the controlled frame is 'right-wrist'
multiplied at the left by **handMgrip**.

```python
# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10
```
This task controls the CoM of the robot by reading the output of the entity **robot.dynamic**.
The second line initialize the output value of signal **robot.dynamic.com**.
It becomes the desired value in the third line.
The control gain is set to 10 in the fourth line.

```python
# --- CONTACTS
#define contactLF and contactRF
contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
contactLF.feature.frame('desired')
contactLF.gain.setConstant(10)
contactLF.keep()
locals()['contactLF'] = contactLF
```
The first line create a set of object necessary to maintain contact with the left-ankle.
The second line specifies the name of the desired feature.
The third line specifies the gain of the contact.
The fourth line maintains the position as a constraint.

```
contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
contactRF.feature.frame('desired')
contactRF.gain.setConstant(10)
contactRF.keep()
locals()['contactRF'] = contactRF
```
The lines specified here are the same than for the previous contact.

```python
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
```
The first line import bindings to the lower C++ framework.
```
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)
```
The first line instantiates a solver to generate a kinematic solver.
The second line defines the size of free variables. 
The third line links the solution of the solver to the input of the robot.    

```python
from dynamic_graph.ros import RosPublish
ros_publish_state = RosPublish ("ros_publish_state")
ros_publish_state.add ("vector", "state", "/sot_control/state")
```
The first line import the python module to publish data in the ROS world (aka topics).
The second line instantiate the object to make the interface from the Stack-Of-Tasks world
to the ROS world. It creates an entity called "ros_publish_state".

The third line adds a topic called **/sot_control/state** from the signal **state**
of the entity **ros_publish_state**


```python
plug (robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal ("ros_publish_state.trigger", 100)
```
The first line connect the signal **robot.device.state** to the the signal **state** of the entity **ros_publish_state**.
The second line calls the signal **ros_publish_state.trigger** at 100 Hz after evaluating the control law.


```python
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
robot.device.control.recompute(0)
```
The first line push the right foot contact task at the top of the SoT.
The second line push the left foot contact task in the SoT.
The third line push the CoM task in the SoT.
The last line ask for a re-computation of the signal named **control** which belongs to the entity **device**.
