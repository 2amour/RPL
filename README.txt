=================================
= First group assignment README =
=================================

Description
===========
This application allows its user to make the ThymioII robot move towards a relative location in its X-Y reference plane avoiding the obstacles that it may encouter in its way.

In order to reach the goal in a robust yet efficient way (given the robot only has partial knowledge of its environment), a tangent bug algorithm is used. This algorithm belongs to the bug algorithms family but provides some optimization respecting to other algorithms of the same family. That is because it can sense a 'safe' point (no obstacle) in front of the robot which can therefore use to get to the goal quicker.

The location is set along with other execution parameters via file parsing and can be precisely reached (considering the odometry is properly calibrated) thanks to a closed loop control which tunes the robot's differential drive's input values, based on the error sensed by the robot's odometry sensors (encoders).

The robot will also show in which step of the tangent bug algorithm is found at any precise moment while performing the task as a color code displayed on the robot's LEDs.

Usage
=====

1. Connect the ThymioII robot to an USB port of the `Blok' wireless pack (Beaglebone running Ubuntu + router + sensor + power bank).
2. Connect to the wireless network of the `Blok' wireless pack and set yourself a static IP which concordes with the computer 'ROS_IP' environmental variable. (SSID: 'rpl_blok', password: 'beaglebone').
3. Launch ROS core in the network master computer.
4. Connect to the Beaglebone via SSH and launch 'wireless_thymio.launch' from 'thymio_navigation_driver'.
5. Open the Eiffel project and compile it.
6. For this application, one command-line argument is needed. This argument is the path to a file which contains the paths to the files containing the parameters which will be parsed during the execution.  Remember to provide it as 'Execution Parameter'.
7. The parameter files found at directory 'thymio_app/' (provided) need also to be properly formatted (key: value) and must contain the desired parameter values for the execution.
8. Execute the 'APP' main application file.

File structure
==============

> Note: '*' means new file, 'm' means modified.

thymio_app
 \_actuator
 |   \_THYMIO_BUTTONS_LEDS
 |   |_THYMIO_CIRCLE_LEDS
 |   |_THYMIO_DIFFERENTIAL_DRIVE
 |   |_THYMIO_SOUND_PLAYER
 |   |_THYMIO_TOP_LEDS
 |
 |_communication
 |   \_THYMIO_TOPICS
 |
 |_control (*)
 |   \_TANGENT_BUG_DRIVE_CONTROLLER (*)
 |   |_TANGENT_BUG_LED_CONTROLLER (*)
 |
 |_io (*)		-- See NOTE below --
 |   \_FILE_PARSER (*)
 |   |_GOAL_PARSER (*)
 |   |_PATHS_PARSER (*)
 |   |_PID_GAIN_PARSER (*)
 |   |_RANGE_SENSOR_PARSER (*)
 |
 |_robot
 |   \_THYMIO_ROBOT (m)
 |
 |_sensor
 |   \_THYMIO_GROUND_GROUP
 |   |_THYMIO_RANGE_GROUP (m)
 |
 |_sequencer (*)
 |   \_TANGENT_BUG_BEHAVIOUR (*)
 |
 |_signaler (*)
 |   \_TANGENT_BUG_SIGNALER (*)
 |
 |_states
 |   \_TANGENT_BUG_STATE (*)
 |       \_FOLLOW_WALL (*)
 |       |_GO_TO_GOAL (*)
 |       |_LEAVE_WALL (*)
 |       |_TANGENT_BUG_STATE (*)
 |       |_UNREACHABLE_GOAL (*)
 |
 |_types
 |   \_ABSTRACT_2D (*)
 |   |   \_POINT_2D (*)
 |   |   |_VECTOR_2D (*)
 |   |
 |   |_LINE_2D (*)
 |   |_POSE_2D (*)
 |
 |_util
 |   \_NON_LINEAR_SPEED (*)
 |   |_PID (*)
 |   |_TIME_PARSER (*)
 |   |_TRANSFORM_2D (*)
 |
 |_APP (m)
 |
 |_thymio_app

*NOTE: A new more reusable parsing system wanted to be implemented to avoid dependencies between domain classes and parsers and between parsers and other parsers but couldn't be fully implemented in time. Therefore, the old system (which is known to work) is provided in this assignment. The other system will be implanted in the following days.

How does it work?
=================

1. First, the main APP receives the command line argument (`paths file' path) and parses the specific execution parameters. Then, it initializes ROS nodes, instantiates the behaviour for the robot to follow and instantiates Thymio robot object assiging it the behaviour. Finally, it calls a Thymio robot's procedure to make the robot start acting as described in the behaviour.

2. The THYMIO_ROBOT object then starts the TANGENT_BUG_BEHAVIOUR with all required robot parts.

3. The behaviour is in charge for the robot movement and the robot LEDs. Therefore, it creates controllers for those tasks (one for the LEDs and two for the robot control) and starts them concurrently.

4. For both controllers we are using a STATE design pattern. Therefore, the controllers call different procedures on the current state and each of the defined states is responsible to implement those features accordingly.

5. The communication between the STATES among themselves and between the contnrollers is done using the signaler TANGENT_BUG_SIGNALER. This signaler contains information related to the current state, the goal, the minimum distance towards measured up to the point, etc.
