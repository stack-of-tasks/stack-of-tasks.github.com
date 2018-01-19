---
layout: page
title: Add your robot
category: Tutorials
---

This tutorial assumes that you have completed the basic tutorials with Romeo.

## Prerequisite

The Stack of Tasks is able to load a robot model from the urdf format.
It is recommanded that the urdf file follows the [REP 120](www.ros.org/reps/rep-0120.html) directives.
The parsing process will look for the following names in the urdf file:
`l_wrist, r_wrist, l_gripper, r_gripper, l_ankle, r_ankle, l_sole, r_sole, BODY, gaze, torso`.

Also, it is mandatory for the base_link to be a virtual joint.

```xml
<link name="base_link">
<joint name="waist" type="fixed">
  <parent link="base_link"/>
  <child link="body"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<link name="body">
```

## Create the associated sot package

To simplify this work, you can start from the [sot-romeo](https://github.com/stack-of-tasks/sot-romeo) package.
This package contains two elements:

- The python files allowing to load the dynamic model
- The C++ files allowing to embed the controller

From now on, we suppose that you have copied the sot-romeo repository into sot-astro (replace astro by the name of your
robot).

### Python
Two python files can be found in the repository sot-astro/src/dynamic_graph/sot/astro

- `__init__.py` (that can remain empty)
- `robot.py` (that specifies some additional values for the urdf file)

Now we are going to adapt the robot.py file to your robot. To this purpose, you need to replace the following pieces of
informations:

1. The name of the package containing your robot
      ```python
      self.urdfDir  = 'package://astro_description/urdf/'
      self.urdfName = 'astro.urdf'
      ```

2. The half-sitting pose

    The half-sitting pose should contains 6+numdofs values.
    The 6 first values correspond to the position of the waist (ie the root joint) in the environment at half sitting,
    then the value are specified by order of depth first in alphabetical order.

3. The sole dimensions

    The sole dimension is not defined in the urdf file.
    To test the pattern generator, it is mandatory to add those values:
    ```python
    self.ankleLength = 0.1935
    self.ankleWidth  = 0.121
    ```

4. (optional) rename the key links.

    This step is not required if your urdf file fits the rep 120 conditions.
    If some of the key frames are not defined, you can complete the jointMap dictionnary
    in order to correct thos missing links
    ```python
    jointMap['torso']   = 'NAME_OF_LINK'
    ```

Once this is done, you can compile and install this package
Simplify the `src/CMakeLists.txt` in order to execute only those commands:

```cmake
INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Install Python files.
SET(PYTHON_MODULE_DIR
${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/astro)
SET(PYTHON_MODULE_BUILD_DIR
${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/astro)

SET(PYTHON_MODULE dynamic_graph/sot/astro)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "robot.py" "${PYTHON_SITELIB}")
```

Add sot-astro to the name of list of the package compiled in install-sot, and install it.

### Testing with the stack of tasks
#### Validation of the urdf reading

Now you can try loading your urdf file with the sot.

Start `roscore`, then open a python terminal, and type these commands:

```python
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph.sot.astro.robot import *
robot = Robot('astro', device=RobotSimu('astro'))
```

If this works, congratulations, the sot was able to load you robot correctly.
Otherwise, you will have to correct the error given. Make sure that the special
frames are specified.

Note: if you have this error `error: Robot has no joint corresponding to waist`, you need to specify the following
element:

```python
jointMap['BODY']
```

#### Creating a launch file to bridge the sot with rviz

First, create the following launch file (e.g. in your robot ros folder):

```xml
<launch>
  <arg name="model" value="$(find astro_description)/urdf/astro.urdf"/>
  <arg name="gui" default="True" />
  <param name="robot_description" textfile="$(arg model)" />
  <param name="use_gui" value="$(arg gui)"/>

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" >
    <env name="ROS_NAMESPACE" value="/dynamic_graph"/>
  </node>
  <node name="robot_pose_publisher" pkg="dynamic_graph_bridge" type="robot_pose_publisher" >
    <env name="ROS_NAMESPACE" value="/dynamic_graph"/>
  </node>
  <node name="rviz" pkg="rviz" type="rviz"/>
</launch>
```

Now that the robot can be parsed, two tests can be run: the kinematics test and the walk test.

#### Kinematic test

To test the kinematics, you can reuse the test presented in the first part of the tutorial:

```python
# 1. Create the robot and the solver
# 1.a Create the robot
from dynamic_graph.sot.astro.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
robot = Robot('astro', device=RobotSimu('astro'))

# 1.b Create a solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize(robot)

# 1.c Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)

# 2. Simulate a loop
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# 3. Add a task of the left wrist
poserpy = PoseRollPitchYawToMatrixHomo ('poserpy')
poserpy.sin.value = (0.18, 0.4, 1, 0,0,0)
plug(poserpy.sout, robot.features['left-wrist'].reference)
robot.features['left-wrist'].selec.value = '000111'
solver.push(robot.tasks['left-wrist'])

go()
```

#### Walk test

The second test simply consists in validating the walk algorithms on your robot.
You can replace the paragraph 3 of the previous example by this one:

```python
# 3. Basic walk
from dynamic_graph.sot.pattern_generator.walking import CreateEverythingForPG, walkFewSteps

CreateEverythingForPG(robot, solver)
walkFewSteps(robot)

go()
```





