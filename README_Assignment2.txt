==================================
= Second group assignment README =
==================================

Description
===========

Mission-Planner
---------------

This application is a higher-level planner for the system. 
It sends a map and goal and start point to the path-planning application through ROS msgs and waits for a path to come back. When the path is comprised of a set of points this mission planner sends sequentially every consequtive pair of goal and start points to the path-planner and then appends the path to the resulting path. 

When the path arrives it gets key-points and sends them sequentially to the robot-controller as way points. Each time the robot is near the way point it updates it to a new one until it reaches goal. 

To apply a D* algorithm the goal and start points are inverted when sent to the planner and the path is reconstructed backwards. If an unknown obstacles is encountered by the sensor then the map is updated and the algorithm is recomputed with a change only in the goal node. 


If the robot senses a obstacle that it was not in the map this application sends




Path-Planning
-------------

This application allows its user to make the ThymioII robot move towards a relative location in its X-Y reference plane avoiding the obstacles that it may encouter in its way.

In order to reach the goal in a robust yet efficient way (given the robot only has partial knowledge of its environment), a tangent bug algorithm is used. This algorithm belongs to the bug algorithms family but provides some optimization respecting to other algorithms of the same family. That is because it can sense a 'safe' point (no obstacle) in front of the robot which can therefore use to get to the goal quicker.

The location is set along with other execution parameters via file parsing and can be precisely reached (considering the odometry is properly calibrated) thanks to a closed loop control which tunes the robot's differential drive's input values, based on the error sensed by the robot's odometry sensors (encoders).

The robot will also show in which step of the tangent bug algorithm is found at any precise moment while performing the task as a color code displayed on the robot's LEDs.



Robot-Controller
----------------



This application executes a path-planning algorihtm. The current implementation uses a graph-search to find the best path between a list of way-points. The algorithm finds sequentially the best path between every pair of consecutive nodes. 

The maps are read from ros /map topic and two different connectivities are implemented: manhattan connectivity (nesw neighbours) or full connectivity (nesw + diagonal neighbours). The obstacles in the map are inflated by a given parameter to allow non-pointwise agents use this algorithm. 

The search order can be specified to be breadth-first-search (FIFO queue), depth-first-search (LIFO stack), or best-first-earch (priority heap). The standard label correcting algorithm is augmented to A* by adding a heuristic function. The heuristic can be one of the following: euclidean distance (2-norm), manhattan distance (1-norm), max coordinate (infinity-norm), triangular heuristic, or zero heuristic (when no cost, A* is identical to standard label correcting algorithm). The edge-cost is also selected from one of this though most probably is given by eculidean distance or manhattan distance. 





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



In a terminal run
	$ roscore 

In another terminal run
	$ rosrun map_server map_server $PATH_TO_YAML_MAP

In a third terminal run 
	$ rosrun rviz rviz

In rviz make sure to plot the map and the path. Visualization markers for start, goal and mid way-points are also published but are not necessary.

Finnaly run the path planning application from the IDE with an execution parameter with "path_to_io/file_paths.txt". From the command line this is
	$ ./path_planner $(PATH_TO_IO)/file_paths.txt








IO Files
=====

Mission-Planner
---------------

Path-Planning
-------------

Robot-Controller
----------------


File structure
==============

How does it work?
=================


Mission-Planner
---------------

Path-Planning
-------------

Robot-Controller
----------------





1. First, the main APP receives the command line argument (`paths file' path) and parses the specific execution parameters. Then, it initializes ROS nodes, instantiates the behaviour for the robot to follow and instantiates Thymio robot object assiging it the behaviour. Finally, it calls a Thymio robot's procedure to make the robot start acting as described in the behaviour.

2. The THYMIO_ROBOT object then starts the TANGENT_BUG_BEHAVIOUR with all required robot parts.

3. The behaviour is in charge for the robot movement and the robot LEDs. Therefore, it creates controllers for those tasks (one for the LEDs and two for the robot control) and starts them concurrently.

4. For both controllers we are using a STATE design pattern. Therefore, the controllers call different procedures on the current state and each of the defined states is responsible to implement those features accordingly.

5. The communication between the STATES among themselves and between the contnrollers is done using the signaler TANGENT_BUG_SIGNALER. This signaler contains information related to the current state, the goal, the minimum distance towards measured up to the point, etc.
