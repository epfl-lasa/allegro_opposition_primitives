allegro_opposition_primitives
=============================
This package implements a grasp controller for opposition primitives. A single opposition primitive represents a subset of grasping patches that are set up into 2 opposing groups for the purpose of applying oppositional pressure on a grasped object. A grasping patch denotes a contiguous surface of the hand acting in unison to exert a grasping force. A grasp can contain one or more opposition primitives.

This package provides a set of predefined grasps, with commands to open and close the hand according to the oppositional intention. Additionally, the sequeezing level along each axis of opposition can be independently or jointly specified.

It is straightforward to define new primitive-based grasps as required.

The package provides a gui interface and a topic-based interface to control the opening/closing of the hand and the squeeze level.

Dependencies
------------

This package depends on the [allegro-hand-ros package][1]. 

[1]: https://github.com/felixduvallet/allegro-hand-ros


Launch
------

The launch file [opposition_controller_with_hand.launch](launch/opposition_controller_with_hand.launch) can be used to launch both the opposition controller node and the underlying allegro pd-control node.

	roslaunch allegro_opposition_primitives opposition_controller_with_hand.launch

Alternatively, if you want the keyboard controls provided by the allegro-hand-ros package, you can start these independently in different terminal windows:

	roslaunch allegro_opposition_primitives epfl_left.launch HAND:=right KEYBOARD:=true
	roslaunch allegro_opposition_primitives opposition_controller_alone.launch






