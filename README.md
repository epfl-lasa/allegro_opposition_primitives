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

	roslaunch allegro_opposition_primitives opposition_controller_with_hand.launch HAND:=right NUM:=1

Alternatively, if you want the keyboard controls provided by the allegro-hand-ros package, you can start the nodes independently in different terminal windows:

	roslaunch allegro_opposition_primitives epfl_hand.launch HAND:=right NUM:=1 KEYBOARD:=true
	roslaunch allegro_opposition_primitives opposition_controller_alone.launch HAND:=right NUM:=1


Gui control
-----------
Run-time control of the chosen primitive is provided through the dynamic reconfigure GUI. After running the allegro_opposition_primitives node, run the dynamic reconfigure client. 

	rosrun rqt_reconfigure rqt_reconfigure 

You should be presented with the following GUI.
![alt tag](img/dyn_reconfig_gui.png)

You can then:
- **chose grasp**
- **open / close** the hand
- **choose the active primitive:** Choosing a number greater than the number of oppositions comprising the grasp will choose all oppositions.
- **specify importance of a primitive** in the grasp.
- **influence speed of closing**
- **vary the squeeze level** 


Topic based control
--------------------
Grasp control is achieved by sending an `std_msgs::String` message on the topic `allegroHand/primitive_control_cmd`. Recognized strings are listed below.
- `grasp <grasp_name>` specifies the grasp to be controlled. Valid grasp names are defined in [primitive_grasp_definitions.yaml](parameters/primitive_grasp_definitions.yaml).
- `home` resets the hand to the default home position. 
- `open`, `close` opens and closes the hand according to the oppositional intentions defined in the grasp.
- `sq <num> <value>` prescribes a squeeze level `value` for the primitive `num`. If `num` is greater than the number of oppositions comprising the grasp, `value` is applied to all oppositions.


Predefined grasps
-----------------
The package predefines a set of grasps which can be selected for execution according to the task being performed. These are shown below.

- put the image here, the human hand demonstrating the grasp. the corresponding grasp with the allegro hand.


Defining new grasps
-------------------






