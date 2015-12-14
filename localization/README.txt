==================================
= Fourth group assignment README =  
==================================


Description
===========

This project implements the particle filter localization algorithm for robots having a laser scanner as sensor and an odometry sensor available. It analyzes this inputs along a reference map and outputs the believed position of the robot. The implementation is dedicated to robots using ROS and uses rviz for visualization purposes.

localization.cpp
-----------------

The main function is implemented in this file. It initializes a ros node that subscribes to three topics, the input map, the laser scan readings and the odometry readings, and creates a publisher that will advertize the result. A localization sequencer is also instantiate. This class has the responsibility to sequence the localization process. 

The map subscriber callback points to a method of the localization sequencer which stores the map. 

The two callbacks of the subscribers to laser scan and odometry readings are synchronized and pointed to a single method called update of the localization sequencer so that both messages are sent at the same time. Once the map is stored and the update method is called the actual algorithm of particle filtering starts.

The parameters needed for running the algorithm are also parsed in this file and retrieved to the localization sequencer which passes them to other members.

localization_algorithm.cpp
--------------------------

The algorithm file implements the LocalizationAlgorithm class which is a virtualized class to make a general interface for localization algorithms. The one implemented here is ParticleFilter algorithm. 

motion_updater.cpp, sensor_updater.cpp, resampler.cpp
-----------------------------------------------------

The algorithm cited above needs at the same time three other algorithms: a motion updater, a sensor updater and a resampler.

This three algorithms are also virtualized to facilitate the implementation of other related
algorithms. The ones implemented in this release are: sample odometry motion model for the 
motion updater, map matching for sensor updater and a roulette wheel sampler for the resampler.
The map matching algorithm is a customized algorithm and doesn't relate to the ones described in the litterature. It is a simplified version of map matching where only the obstacles seen are mapped to the map. The correlation is given simply by the number of matches between sensed points by sensor and real obstacles in the map.

The motion updater also needs a sampler method that is actually implemented by yet another virtualized class ProbabilityDistribution. Childs of this class are so far only a TriangularDistribution and ApproxNormalDistribution.

All this algorithms are actually created on the user demand through parsing parameters such as the type of algorithm and other parameters. So the user can set up a triangular distribution sampler or an approximate normal distribution before running the code and without needing to recompile.

pose.cpp
--------

The types used in this application for poses and other specific structures are implemented in this file. An effort has been made to try to template the Pose and Particle structures but hasn't been particularly successfull for the dimensionality abstraction, meaning setting a 2D or 3D pose for example. Pose type has been templated and with overloaded constructors it let's the programmer set the dimension of the node it needs. For further development it would be wiser to explore the Eigen library and use it instead of rebuilding the wheel.

visualizer.cpp
--------------

The visualizer class is needed to visualize the output of the algoritm. It stores a publisher and publishes a message whenever its method publishParticles is called.

parser_sequencer.cpp
--------------------

This file contains a class which is responsible of parsing the parameters. There are different functions implemented which specifically parse and instantiate parameters for the program.

localization.launch
-------------------

This file contains a set of commands for ROS which will load the parameters .yaml files, run rviz for visualization and play a rosbag stored as an example of usage.

Usage
=====

1. Set the necessary parameters in the .yaml files in the parameters directory.

2. Run the command 'roslaunch localization localization.launch' which will run rviz, a rosbag with a replay of the movement of a robot, sensed laser scan and a map. Also the node localization will be initialized with the above functionality.

File structure
==============

Localization
------------
localization
 \_include
 | \_algorithms
 | | \_localizer.h 							
 | | |_motion_updater.h
 | | |_resampler.h
 | | |_sensor_updater.h
 | |
 | \_io
 | | \_parser_sequencer.h
 | | 
 | \_parameters
 | | \_localizer_parameters.h
 | | |_motion_updater_parameters.h
 | | |_parameters_bag.h        
 | | |_resampler_parameters.h
 | | |_sensor_updater_parameters.h
 | | |_topics_parameters.h
 | | 
 | \_probability_distributions
 | | \_probability_ditributions.h
 | |  
 | \_sequencer
 | | \_pose.h
 | | 
 | \_visualizer
 |  \_visualizer.h
 |
 |_src
 | \_algorithms
 | | \_localizer.cpp
 | | |_motion_updater.cpp
 | | |_resampler.cpp
 | | |_sensor_updater.cpp
 | |
 | \_io
 | | \_parser_sequencer.cpp
 | | 
 | \_parameters
 | | \_localizer_parameters.cpp
 | | 
 | \_probability_distributions
 | | \_probability_ditributions.cpp
 | |  
 | \_sequencer
 | | \_pose.cpp
 | | 
 | \_visualizer
 |  \_visualizer.cpp
 |
 \
  \_localization.cpp
