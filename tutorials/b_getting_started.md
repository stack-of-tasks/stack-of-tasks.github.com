---
layout: page
title: Getting Started
category: Tutorials
---

## Robot kinematic simulations

We assume that:

1. the stack of tasks has been installed using the installation instruction provided [there](download);
2. you understood the [dynamic graph mecanism](tuto_a_dynamic_graph).

In this section are detailed some examples to test the stack of tasks framework on the humanoid robot Romeo.

To vizualize the robot, you need `rviz`.

### Running the python scripts

To execute the scripts and display the results with rviz, you need 2 consoles.

In the first, start the viewer: `roslaunch romeo_description sot_display.launch`

Note: if you do not want to use the viewer, you can only start `roscore`
Of course, the ros environment must have been loaded to call rviz.

In the second, start the python script: `ipython -i kine_romeo.py`

To run them, it is necessary to complete the environment variables `LD_LIBRARY_PATH` and `PYTHONPATH`.
To this purpose, you can either

- Source the file `config_XXX.bash` created during the compilation using the `install-sot.sh` script. This file is
  located in the same folder as `install_sot.sh`.
- Complete manually the environment variables:
  ```bash
  export your_installation_path=${HOME}/devel/INSTALLATION_FOLDER/install
  export ROS_PACKAGE_PATH=${your_installation_path}/../:$ROS_PACKAGE_PATH;
  export LD_LIBRARY_PATH=${your_installation_path}/lib/:$LD_LIBRARY_PATH;
  export LD_LIBRARY_PATH=${your_installation_path}/lib/plugin:$LD_LIBRARY_PATH;
  export PYTHONPATH=${your_installation_path}/lib/python2.7/site-packages/:$PYTHONPATH
  ```

Note:
The tutorial scripts are usually located in `${your_installation_path}/lib/python2.7/site-packages/dynamic_graph/tutorial`.
With the installation script, the installation path is `${HOME}/devel/INSTALLATION_FOLDER/install`,
where `INSTALLATION_FOLDER` is the folder specified in command line when running the compilation script.


## `kine_romeo.py`

The simplest example to try is `kine_romeo.py`.
Here is the script split up:

```python
# 1. Create the robot and the solver
# 1.a Create the robot
from dynamic_graph.sot.romeo.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
robot = Robot('romeo', device=RobotSimu('romeo'))

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
[go,stop,next,n] = loopShortcuts(runner)

go()

# 3.
from dynamic_graph.sot.core import *
from dynamic_graph import plug

robot.comTask.controlGain.value = 200
robot.tasks['left-ankle'].controlGain.value = 200
robot.tasks['right-ankle'].controlGain.value = 200

solver.push(robot.tasks['left-wrist'])
#robot.featureCom.selec.value = '011'
# poserpy = PoseRollPitchYawToMatrixHomo('poserpy')
# poserpy.sin.value = (0.08, 0.4, 0.0, 0,0,0)
# plug(poserpy.sout, robot.features['left-wrist'].reference)
```

This code can be split in 3 parts.

### Part 1: Create the robot and the solver

```python
# 1. Create the robot and the solver
# 1.a Create the robot
from dynamic_graph.sot.romeo.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
robot = Robot('romeo', device=RobotSimu('romeo'))
```

This creates the dynamic model of the robot, as well as the device entity.
The dynamic entity allows the computation of the jacobian, inertia, com of the robot.
The device entity simulates the behavior of the robot.


```python
# 1.b Create a solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize (robot)
```

This creates the default stack of tasks solver, that you can call using `solver`.
You can add or remove tasks from it.
To have a finer access to the tasks manipulation, you have to work directly
with the entity `solver.sot`.

```python
# 1.c Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)
```

This creates a ROS binding allowing to display the model of the robot in rviz.
It is not mandatory.

#### Part 2: realizing a virtual loop

```python
# Part 2: realizing a virtual loop
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n] = loopShortcuts(runner)

go()
```

When the controller is not embedded, it is necessary to trigger regularly the recomputation of the dynamic graph.
This is realized here by the line `robot.device.increment(dt)` that increments its time value and realizes the
kinematic integration of the control law.

Four methods control this loop:

- `go` starts the loop
- `stop` stops the loop
- `next`
- `n`

#### Part 3: specify the task

At this point of the code, the simulation is already running, but the robot is still. It is controlled by 3 tasks:

```python
print solver
+-----------------
+   SOT
+-----------------
| romeo_task_com
| romeo_task_left-ankle
| romeo_task_right-ankle
+-----------------
```

- `romeo_task_com`, A task controlling the center of mass
- A 6d position task for the left ankle
- A 6d position task for the right ankle

Those three tasks are part of the set of tasks predefined (but not added to the stack) in the dictionary of tasks
provided by the entity `_Robot_`.
The dictionary can be consulted using the command: `robot.tasks`
All of those tasks are 6d tasks.

```python
In [1]: robot.tasks
Out[1]:
{'balance': Task romeo_task_balance:
--- LIST ---
-> romeo_feature_com
-> romeo_feature_left-ankle
-> romeo_feature_right-ankle
,
 'left-wrist': Task romeo_task_left-wrist:
--- LIST ---
-> romeo_feature_left-wrist
}
```

Let's now add a task for the left wrist and define its objective.
As a 6d task, it is possible to define the objective by providing directly the homogeneous matrix.
To simplify its definition, we are going to use an entity that converts 6d vectors (x,y, z, roll, pitch, yaw) into homogeneous matrices:
```python
poserpy = PoseRollPitchYawToMatrixHomo('poserpy')
poserpy.sin.value = (0.08, 0.4, 0.0, 0, 0, 0)
plug(poserpy.sout, robot.features['left-wrist'].reference)
solver.push(robot.tasks['left-wrist'])
```

In the script, only the position of the hand is controlled. To change the degrees of freedom controlled, change the value of the signal selec.
This signal is a sequence of boolean in **reverse polish notation**: psi, theta, phi, z, y, x. For example:

```python
robot.features['left-wrist'].reference.selec.value = '111111' # The task is controlled in position and orientation.
taskRH.feature.selec.value = '000111' # The task is then controlled in position only
```

This task is characterized by 4 entities: the task, the operational point (here the right hand augmented by a translation), the desired position for the hand (here only 3d are defined) and the gain of the task (an adaptive gain).
All those elements are gathered in a structure called _taskRH_,instance of _MetaTask6d_, that realizes all the links between the several entities.

The definition of this task does not use the predefined version of the task contained in the dictionary, only to present an example of task creation.

Typing `go` starts a the simulation loop.
The robot will then go to the desired positions defined by the tasks.
You can change interactively the content of the stack, the hand... using the python interface.

To continue the example, you can for example try adding a task on the left hand, following the same scheme as for the right hand.

## `walk_romeo.py`

In this example, the Romeo robot goes for a (kinematic) walk.
This script uses the jrl-walkgen package to compute the required postures for the feet, the center of mass and the waist orientation.

The script is built in the same way than the kinematic example (Creation of the solver, definition of the *incrementation* loop),
and definition of the pattern generator elements:

```python
from dynamic_graph.sot.pattern_generator.walking import CreateEverythingForPG , walkFewSteps, walkAndrei
CreateEverythingForPG(robot, solver)
# walkFewSteps(robot)
walkAndrei(robot)
robot.pg.velocitydes.value = (0.1, 0.0, 0.0)
```

The stack contains the following tasks (by order of decreasing priority)

- `ROMEO_task_com`, that forces the Center of mass of the robot to be in the support polygon (only the x and y axes are
  constrained). You can access it using `robot.tasks['com']`.
- `taskWaist`, that constrains the rotation and translation of the waist in a plane. You can access it using
  `robot.tasks['waist']`.
- `ROMEO_task_right-ankle`, that defines the 6 dof position of the right Foot
- `ROMEO_task_left-ankle`, that defines the 6 dof position of the left Foot

In addition to those tasks (that translate the output of the pattern generator into tasks), another task is defined:
`robot.tasks['robot_task_position']`, to force the arms, head and chest to stay still during the walk

You can control the walk of the robot with the following command: `robot.pg.velocitydes.value = (x, y, theta)`

- x is the forward/backward velocity
- y is the lateral velocity
- theta is the rotation velocity (requires that ||(x,y)|| > 0.000  1)
