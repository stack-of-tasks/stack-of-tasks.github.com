---
layout: page
title: Embedding the controller
category: BackupTutorials
---

Porting the code to the physical platform is likely to require you to wrap the controller in an external piece of
software. For example,for the HRP robots, it is required to call the controller in a openrtm component. See slides 42
for more details on this.

The purpose of using the embedding the controller is to retrieve the data provided by the real robot. This may require
the definition of an appropriated device entity.

Embedding the controller present the following differences:

1. The incrementation of the command law is realized automaticatilly by the controller. Hence you must not define the incrementation thread in the python script.
2. In order to retrieve a python interpreter, you can run the following command: `rosservice call run_command`
3. Before starting the controller, it is *mandatory* for the solver to be defined and plugged to the device
4. Using an embedded controller allows to realize simulation with ros and call python instruction from ros.

## Executing python commands via ROS

Since the controller is embedded, you have to open a remote interpreter in your terminal in order to execute python commands.
The remote interpreter provides the same output as the classical interpreter, but does not allow the autocompletion.
`rosservice call run_command`

If you run this command with an argument, you will only execute the command you provided, without opening the interpreter.
`rosservice call run_command  "print 'hello world'"`

You can also execute a file, using this command
`rosrun dynamic_graph_bridge run_command '/absolute/path/to/file.py'`

Note: The `run_command` service only considers the first line of your command, and does not accept end of line characters.
In other words, if you run this:
`rosservice call run_command "print 'a'\n print 'b'"`
You'll have this result:
```
result: <NULL>
stdout: ''
stderr: SyntaxError: ('unexpected character after line continuation character', ('<string>', 1, 21, "print 'a'\\n print 'b'"))
```

But if you use this:
`rosservice call run_command "print 'a'; print 'b'"`

You'll have the expected result:
```
result: None
stdout: a
b
```
