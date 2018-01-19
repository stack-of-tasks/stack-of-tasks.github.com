---
layout: page
title: Bridge with ROS
category: Tutorials
---

The core of the stack of tasks mechanism is independent from ROS: an execution is hence realized in a closed environment
(the entities are only defined in the python environment and are only accessible via python commands).
However, some bridges with ROS exist: for the display, but also to send and receive some pieces of data.


## Signal to ROS message

Two entities allow you to exchange data from the ROS environment to the stack of tasks one,
and convert ros messages in sot signals, namely `rosPublish` and `rosSubscribe`.

Here is a example of publication of matrix homogeneous from the sot environment to the ROS environment:

```python
ros.rosPublish.add('matrixHomoStamped', robot.frames['rightGripper'].name +'_position', 'right_gripper')
ros.rosPublish.displaySignals()
--- <rosPublish> signal list:
    |-- <Sig:N12dynamicgraph10RosPublishE(rosPublish)::input(MatrixHomo)::romeo_rightHand_position (Type Cst) AUTOPLUGGED
    `-- <Sig:N12dynamicgraph10RosPublishE(rosPublish)::input(int)::trigger (Type Fun)
```

```bash
rostopic list
/right_gripper
```

This creates an input signal for the ros.rosPublish entity (`romeo_rightHand_position`), as well as the rostopic
element `/right_gripper`.

Please note that this alone does **NOT** publish anything. In order to publish something, it is hence necessary to a signal to the one created in rosPublish.

```python
plug(robot.frames['rightGripper'].position, ros.rosPublish.signal(robot.frames['rightGripper'].name +'_position'))
```

Now you are publishing:
```bash
rostopic echo -n 1 /right_gripper
header:
  seq: 24577
  stamp:
    secs: 1408440100
    nsecs: 786866991
  frame_id: /dynamic_graph/world
child_frame_id: ''
transform:
  translation:
    x: 0.12276189235
    y: -0.242369571396
    z: 0.659561031231
  rotation:
    x: -0.0883991792798
    y: 0.534618973732
    z: -0.0459091290832
    w: 0.839202284813
```

More details are provided [on the ROS wiki](http://wiki.ros.org/dynamic_graph_bridge).

## ROS tf to signal
Importing a tf requires to convert first the tf into a matrix homogeneous in order to import it in the stack of tasks.
The child_frame HAS TO be the odom frame, which is the base for the SoT environment.
Hence, in order to use the position of an object in the stack of tasks, you can add
this to the launch file:

```xml
<node name="object"    pkg="dynamic_graph_bridge" type="tf_publisher">
  <param name="frame"       type="string" value="object" />
  <param name="child_frame" type="string" value="odom" />
  <param name="topic"       type="string" value="object" />
</node>
```
