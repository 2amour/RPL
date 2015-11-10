==================================
= Second group assignment README =
==================================

Description
===========

This group of applications can be used for allowing a ThymioII robot move towards a point in its plane space avoiding both known (by source of a map) and unknown obstacles. In order to try to increase modularity and reusability and also so independent tasks can be executed in different computers, it has been decided to separate responsabilities in different Eiffel applications. The communication between the applications is done by means of ROS topics and has been designed in terms of modularity.

A scheme of how the communication is performed can be seen in the file 'rosgraph.png'.

Following, a description for the parts can be read:

Mission-Planner
---------------

This application is a higher-level planner for the system. 
It sends a map and goal and start point to the path-planning application through ROS msgs and waits for a path to come back. When a set of points must be visited this mission planner sends sequentially every consequtive pair of goal and start points to the path-planner and then appends the path to the resulting path. 

When the path arrives it gets key-points and sends them sequentially to the robot-controller as way points. Each time the robot is near the way point it updates it to a new one until it reaches goal. 

The goal and start points are inverted when sent to the planner and the obtained path is reconstructed backwards. 
If the robot senses a obstacle that it was not in the map this application sends a new map to the path planner and a new starting point (the actual point) for the algorihtm. 

Path-Planning
-------------

This application executes a path-planning algorihtm. The current implementation uses a graph-search to find the best path between a two points. The algorithm finds sequentially the best path between every pair of consecutive nodes. 

The maps are read from ROS topic and two different connectivities are implemented: manhattan connectivity (nesw neighbours) or full connectivity (nesw + diagonal neighbours). The obstacles in the map are inflated by a given parameter to allow non-pointwise agents use this algorithm. 

The search order can be specified to be breadth-first-search (FIFO queue), depth-first-search (LIFO stack), or best-first-earch (priority heap). The standard label correcting algorithm is augmented to A* by adding a heuristic function. The heuristic can be one of the following: euclidean distance (2-norm), manhattan distance (1-norm), max coordinate (infinity-norm), triangular heuristic, or zero heuristic (when no cost, A* is identical to standard label correcting algorithm). The edge-cost is also selected from one of this though most probably is given by eculidean distance or manhattan distance. 

The number and size of cells of input map can be reduced using the cell size parameter. This allows computing and approximate path between the given points much faster.

The input points are received using a ROS topic and the output path between the nodes is also published using a ROS topic.

Robot-Controller
----------------
This application makes the robot move towards a given point avoiding the obstacles that it may encouter in its way.

In order to reach the goal a tangent bug algorithm is used. The location of the goal is received from the mission-planner while position and heading are retrieved to the latter. This communication enables the robot to follow a path. 

Other parameters are set via file parsing.

A PID closed loop control tunes the robot's drive input values based on the error sensed by the robot's encoders with respect to the current goal.


Usage
=====

1. Connect the ThymioII robot to an USB port of the `Blok' wireless pack (Beaglebone running Ubuntu + router + sensor + power bank).

2. Connect to the wireless network of the `Blok' wireless pack and set yourself a static IP which concordes with the computer 'ROS_IP' environmental variable. (SSID: 'rpl_blok', password: 'beaglebone').

3. Launch ROSCORE in the network master computer.
	$ roscore 

4. Connect to the Beaglebone via SSH and launch 'wireless_thymio.launch' from 'thymio_navigation_driver'.
	$ roslaunch thymio_navigation_driver wireless_thymio.launch

5. In the network master run. 
	$ roslaunch roboscoop_ros thymio_driver.launch

6. Yet in another terminal run
	$ rosrun map_server map_server $PATH_TO_YAML_MAP

7. Run the Mission Planner node
	$ ./mission_planner mission_planner_parameters_file topics_file

8. Run the Path Planner node
	$ ./path_planner algorithm_parameters_file map_parameters_file topics_file

9 Run the Robot Controller node
	$ ./robot_controller file_path

NOTE: most of this could be launched as a roslaunch file. 



IO Files
========

Mission-Planner
---------------

mission_planner_parameters.txt
	List of points to visit and threshold to move between way points

mission_planner_topics.txt
	List of ROS topics and ROS node name

Path-Planning
-------------

path_planner_params.txt
	Path frame id, movement and heuristic cost and search strategy

path_planner_topics.txt
	List of ROS topics and ROS node name

map_params.txt
	Grid cell size (in pixels, for blocking), obstacle inflation rate and connectivity strategy

Robot-Controller
----------------
files_parameters.txt
	File with the paths of the other files with parameters.

goal_parameters.txt
	Coordinates of goal point and reached point threshold.

range_sensors_parameters.txt
	Sensors parameters.

pid_parameters.txt
	PID gains.

wall_following_parameters.txt
	Parameters for following an obstacle.



File structure
==============

Mission-Planner
---------------
mission_planner
 \_actuator
 |   \_PATH_PUBLISHER 						(ROS publisher for a path)
 |   |_OCUPANCY_GRID_PUBLISHER 				(ROS publisher for a map)
 |   |_POINT_PUBLISHER						(ROS publisher for a point)
 |
 |_controller
 |   \_MISSION_PLANNER_CONTROLLER 			(assynchronously execute the path request, path processing and target point updates)
 |
 |_io 
 |   \_FILE_PARSER 							(deferred parser for a txt file)
 |   |_MISSION_PLANNER_TOPICS_PARSER		(parser for the mission planner ROS topics)
 |   |_MISSION_PLANNER_PARAMETERS_PARSER 	(parser for the mission planner PARAMETERS)
 |   |_TOPIC_PARAMETERS_FILE_PARSER 		(deferred parser for a txt file containing parsrs)
 |_parameters
 |   \_MISSION_PLANNER_PARAMETERS_BAG		(class with classes of mission planner parameters)
 |   |_MISSION_PLANNER_PARAMETERS			(class with mission planner parameters)
 |   |_MISSION_PLANNER_TOPICS_PARAMETERS	(class with mission planner topics parameters)
 |   |_PARAMETERS_BAG						(deferred class of classes of parameters)
 |   |_PARAMETERS							(deferred class of parameters)
 |   |_TOPIC_PARAMETERS						(deferred class of topic parameters)
 |
 |_ros
 |  \_NAMED_ROBOSCOOP_NODE					(class that implements a named roboscoop node)
 |  |_ROS_NAMED_NODE_STARTER				(class that launches the roboscoop node)
 |
 |_sequencer 
 |   \_MISSION_PLANNER_BEHAVIOUR 			(a sequencer to execute the application of the class asynchronously)
 |
 |_signaler 
 |   \_MISSION_PLANNER_BEHAVIOUR 			(a signaler that contains the way points and the resulting path)
 |   |_PATH_SIGNALER_WITH_FLAG 				(a subscriber to a ROS point msg.)
 |	 |_POINT_LISTENER						(a deferred subscriber to a ROS point msg.)
 |   |_POINT_SIGNALER						(an subscriber to a ROS point msg.)
 |
 |_types
 |   \_POINT 								(utility class for implementing a point)
 |
 |_APP (main function)
 |
 |_mission_planner

Path-Planning
-------------
path_planner
 \_actuator
 |   \_PATH_PUBLISHER 				(ROS publisher for the path obtained)
 |
 |_connectivity_strategies
 |   \_FULL_CONNECTIVITY_STRATEGY		(connectivity with NESW and diagonal neigbours)
 |	 |_MANHATTAN_CONNECTIVITY_STRATEGY 	(connectivity with NESW neighbours)
 |
 |_controller
 |   \_PATH_PLANNING_CONTROLLER 	(controller that executes graph search algorithm between every consecutive pair of way points and publishes path)
 |
 |_graph_search_strategies
 |   \_LABEL_CORRECTING_GRAPH_SEARCH (label-correcting algorithm class)
 |       \_A_STAR 					(A* implementation of label-correcting algorithm)
 |       |_BFS						(BFS implementation of label-correcting algorithm)
 |       |_DFS 						(DFS implementation of label-correcting algorithm)
 |
 |_heuristic_strategies
 |   \_EUCLIDEAN_HEURISTIC 			(euclidean distance heuristic)
 |   |_INFINITY_NORM_HEURISTIC 		(max (#x, #y) heuristic)
 |	 |_MANHATTAN_HEURISTIC			(manhattan distance heuristic)
 |   |_TRIANGLE_HEURISTIC			(1 if , sqrt(2) if diagonal heuristic)
 |	 |_ZERO_HEURISTIC				(zero cost heuristic)
 |
 |_io 
 |   \_PARAMETERS_FILE_PARSER				(a deferred parameters file parser)
 |       \_MAP_PARAMETERS_PARSER 			(a parser for map processing parameters)
 |       |_PATH_PLANNER_PARAMETERS_PARSER	(a parser for path-planning algorithm parameters)
 |       |_TOPIC_PARAMETERS_FILE_PARSER 	(a deferred parser for ROS topics parameters)
 |           \_PATH_PLANNER_TOPICS_PARSER 	(a parser for path-planning ROS topics parameters)
 |
 |_parameters
 |   \_PARAMETERS								(generic parameters class)
 |   |   \_MAP_PARAMETERS						(map processing parameters)
 |   |   |_PATH_PLANNER_PARAMETERS				(path planner parameters)
 |   |   |_TOPIC_PARAMETERS						(deferred topic container class for ROS topics parameters)
 |   |       \_PATH_PLANNER_TOPICS_PARAMETERS	(path planner topics parameters)
 |   |_PARAMETERS_BAG							(deferred class that contains different types of parameters)
 |       \_PATH_PLANNER_PARAMETERS_BAG			(path planner parameters bag that groups different types of parameters)
 |
 |_ros
 |   \_NAMED_ROBOSCOOP_NODE			(a Roboscoop node with a name, which manages external communication)
 |   |_ROS_NAMED_NODE_STARTER		(tarter for an external Roboscoop named node)
 |
 |_sequencer 
 |   \_PATH_PLANNING_BEHAVIOUR 		(a sequencer to execute the search algorithm and publish start, goal and middle way points)
 |
 |_signaler (*)
 |   \_MAP_PARAMETERS_SIGNALER 		(a signaler that contains the map connectivity and the inflation parameter)
 |   |_PATH_PLANNING_SIGNALER 		(a signaler that contains the way points, the edge cost, heuristics for a* algorithm, and which search order is used)
 |   |_POINT_LISTENER		 		(a signaler that can be used for receiving a point through a ROS topic)
 |   |_POINT_SIGNALER		 		(a signaler that contains the current state of a point)
 |
 |_types
 |   \_POINT						(a 3D point representation)
 |
 |_util
 |   \_GRID_FROM_MAP 				(utility class for creating a grid from a nav_msgs/occupancyGrid ROS msg)
 |   |_LABELED_NODE 				(utility class for graph-labeling algoritm that contains a node and a label assigned by the latter)
 |
 |_APP (main function)
 |
 |_path_planner

Robot-Controller
----------------
robot_controller
 \_actuator
 |   \_ODOMETRY_PUBLISHER 		(publishes odometry msgs to ROS)
 |   |_POINT_PUBLISHER 		(publishes point msgs to ROS)
 |   |_THYMIO_DIFFERENTIAL_DRIVE 		(controls the speed of the robot)
 |
 |_communication
 |   \_THYMIO_TOPICS  		(list of used ROS topics for interaction with Thymio robot)
 |
 |_control
 |   \_TANGENT_BUG_CONTROLLER  		(controls robot during tangent bug behaviour)
 |
 |_io
 |   \_FILES_PARAMETERS_FILE_PARSER 		(files paths file parser)
 |   |_GOAL_PARAMETERS_FILE_PARSER 		(goal file parser)
 |   |_PARAMETERS_FILE_PARSER 		(parameters file parser)
 |   |_PID_PARAMETERS_FILE_PARSER 		(pid file parser)
 |   |_RANGE_SENSORS_PARAMETERS_FILE_PARSER  		(range sensor file parser)
 |   |_WALL_FOLLOWING_PARAMETERS_FILE_PARSER  		(Wall following parameters file parser)
 |   |_TOPIC_PARAMETERS_FILE_PARSER	(topic file parser)
 |   |_ROBOT_CONTROLLER_TOPICS_PARSER 		(topic file parser)
 |
 |_parameters
 |   \_FILES_PARAMETERS		(files paths file parser)
 |   |_GOAL_PARAMETERS 		(goal file parser)
 |   |_PARAMETERS 		(generic parameters class)
 |   |_PARAMETERS_BAG		(generic class that contains different types of parameters)
 |   |_PID_PARAMETERS		(PID controller parameters)
 |   |_RANGE_SENSORS_PARAMETERS  		(horizontal sensors parameters)
 |   |_WALL_FOLLOWING_PARAMETERS  		(obstacle following parameters)
 |   |_TOPIC_PARAMETERS				(deferred topic container class for ROS topics.)
 |   |_ROBOT_CONTROLLER_TOPIC_PARAMETERS 	(robot controller ROS topics)
 |
 |_robot
 |   \_THYMIO_ROBOT 		(robot thymio-II)
 |
 |_sensor
 |   \_RANGE_GROUP 		(interface for data gathered by a group of distance sensors)
 |   |	\_THYMIO_RANGE_GROUP  		(group of Thymio's horizontal range sensors)
 |   |	
 |   |_THYMIO_GROUND_GROUP 		(group of Thymio's ground range sensors)
 |
 |_sequencer 
 |   \_THYMIO_BEHAVIOUR 		(behavior of a robot)
 |   |_TANGENT_BUG_BEHAVIOUR  		(behaviour of the robot implementing tangent bug algorithm)
 |  
 |_signaler 
 |   \_TANGENT_BUG_SIGNALER  		(state of tangent bug)
 |   |_POINT_LISTENER			(class for receiving a point)
 |   |_POINT_SIGNALER 		(current state of the point)
 |
 |_states
 |   \_TANGENT_BUG_STATE  		(states of the tangent bug algorithm)
 |       \_AT_GOAL 		(robot reached goal)
 |       |_GO_TO_GOAL 		(robot goes to goal)
 |   	 |_FOLLOW_WALL 		(robot follows an obstacle)
 |   	 |_LEAVE_WALL 		(robot leaves an obstacle to go to goal)
 |   	 |_UNREACHABLE_GOAL 		(goal is not reachable)
 |
 |_types
 |   \_GEOMETRY_2D  		(geometry for 2D space)
 |   |   \_POINT_2D  		(point in 2D space)
 |   |   |_VECTOR_2D  		(vector in 2D space)
 |   |
 |   |_LINE_2D  		(line in 2D space)
 |   |_POSE_2D  		(pose in 2D space)
 |
 |_util
 |   \_NON_LINEAR_SPEED  		(linear speed control)
 |   |_PID  		( pid controller)
 |   |_TIME_HANDLER 		(timing class)
 |   |_TRANSFORM_2D  		(coordinate system transformation in 2D space)
 |
 |_APP 		
 |
 |_robot_controller



How does it work?
=================

Mission-Planner
---------------
The mission planner executes in parallel the following behaviours:

1. When a obstacle is sensed propioceptively by the robot it checks if it is a known obstacle and if it is not it updates the map. 
2. It updates the goal of the robot-controller. It changes the way-points when the position of the robot is near it or when the way point is inside an unkown obstacle. 
3. It sends to the path-planner a map, a start and a goal point. The start and goal point are consequitve way points of the desired path. 
4. It recieves the resulting path and extracts key-points of this path to send as goal positions to the robot driver. 

Path-Planning
-------------
1. The app.e class parses the command-line arguments with the paths to the different parameters files. For each of the paths it parses the convenient parameters. Then, it initializes the ROS node, creates the path planning behaviour and initiates it.

2. The behaviour creates the used signalers (path planning algorithm parameters, map related parameters and start and goal points), initializes a listener to the map topic, initializes the listeners for start and goal points to corresponding topics and initializes the publisher to the path topic. It waits then until it recieves a msg from the map topic and the start and stop topics. It calls asynchronously the controller that executes the path_planning algorithm with the search feature.

3. The controller waits until a msg from map topic and a pair of points are published and creates a grid_graph from the msg read. Then according to the search strategy selected it initializes the open bin to an arrayed_queue in the case of breadth-first-search, an arrayed_stack in the case of depth-first-search, or a heap_priority_queue in the case of best-first-search. Then it executes the graph-search algorithm. When this algorithm terminates it publishes the optimal path for every iteration.

4. The labeling correcting algorithm works as:
	- Initialize parent_strucutre as a hash table which keys are nodes and values are parents
	- Add the starting node to the open set
	- Initialize the labels of every node to +infinity
	- Loop until the open set is empty
		- Get the last_item from the open set
		- For every neighbour of the last_item do:
			new_label := last_item_label + cost(last_item, neighbour)
			1) new_label < old_label
			2) new_label + heuristic (neighbour, goal) < goal_label
		- if 1) AND 2) hold, add neighbour to open_set and update parent[neighbour] = last_item
		- remove last_item from open set
	- When loop is finished recover path by setting key = goal and recursively transversing parent[key] = new_key until new_key = start.

5. The path is recomputed every time new points or a new map is received.

Robot-Controller
----------------
1. First, the app.e class receives the command line argument (`paths file' path) and parses the specific execution parameters. Then, it initializes a ROS node and instantiates the tangent bug behaviour for the robot. Then it instantiates Thymio robot object assigning it the behaviour. Finally, it calls a Thymio robot's procedure to make the robot start acting as described in the behaviour.

2. The THYMIO_ROBOT object starts the TANGENT_BUG_BEHAVIOUR with all required robot parts. The latter listens to the goal sent by the mission planner and executes a controller that drives the robot to the goal. At the same time it publishes any obstacle it senses so that mission planner takes them into account for following calculations. It also publishes odometry information for mission planner that keeps a two way communication, receiving goals and sending position and heading. 

3. This behaviour runs a controller which updates the velocity of the robot according to its environment. If an obstacle is sensed in the way to the given goal point it will follow it until there is a safe point where it can leave the obstacle and get closer to the goal point. If no safe point is found after making a loop around the object, the robot will stop and precise that the goal is unreachable. When the robot gets close to the goal it will also stoThe communication between the STATES among themselves and between the controllers is done using the signaler TANGENT_BUG_SIGNALER. This signaler contains information related to the current state, the goal, the minimum distance towards measured up to the point, etc.
