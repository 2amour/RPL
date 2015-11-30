======================================
= Thrid individual assignment README =
======================================
File Structure
===========
DOXYGEN comments together with the file structure were produced for this project. Please find the html under ./object_recognition/html/index.html

Description
===========
This application executes a object recognition algorithm. The current implementation uses spin-image descriptors from a point-cloud data to compare with a given set of models. 
It has two modules: a training and a recognition module. 

The training module provides a list of spin-images of a list of models per category and a single spin images formed by stacking all spin images into a single spin image.
The recognition module publishes a marker of a given color for the recognized category. 
One must provide a list of categories with its spin-images and, after filtering and clustering, the algorithm compares each cluster to each category. 
When a match is found the marker is published and when none is found an unknown marker is published.

The number of categories currently implemented are ducks and humans but the implemention allows to add a new class seamlessly. 

Input Parameters
================
The user can input parameters through a text file or through rosserver parameters. 
The yaml file has a dictionary with the key being the filter, segmentation, descriptor or correlation strategy to be used and as value the parameters to be used. 

When the yaml file is used the filters (if present) are applied in the next order: Pass Through filter, Statistical Outlier filter, Voxel Grid filter. 

Usage
=====

Training module
---------------
Run:
	$ rosrun object_recognition spin_training file_with_paths_to_training_models output_path
	- or
	$ roslaunch object_recognition spin_training.launch class:=duck (or class:=person)

Roslaunch Parameters:
	- output (default = screen), where the output is printed to a log file or to the screen. 
	- class (default = duck), which category is going to be trained.  

Recognition module
------------------
Run:
	$ rosrun object_recognition spin_recognition file_with_classes
	- or
	$ roslaunch object_recognition spin_recognition.launch 

Roslaunch Parameters:
	- output (default = screen), where the output is printed to a log file or to the screen. 
	- rviz (default = true), whether to launch or not rviz.  
	- model_path (default = "$(find object_recognition)/image_classes/stack_classes.txt" ), file with paths to models. 

How does it work?
=================

Training module
---------------
1. Descriptor Generator
	For every image in the model the spin-image is generated and saved. 

2. Stacking
	A stack of spin-images is formed from the spin-images of every image and saved into a stack. 

Recognition module
------------------
1. Filtering
	A list of filters is set by the user and these are implemented in order to the input cloud. 
	One can implement one or more of the same or different classes of filters. The current implemented filters are:
		- Pass-through filter.
		- Statistical outliers filter.
		- Voxel grid downsampler.
		- Extract indices filter. (Used only to extract clusters in point 2).
		
2. Clustering
	A single clustering strategy set by the user is implemented to the resulting cloud obtained after the sequence of applied filters. 
	Each of the obtained clusters is then recognized. 
	The current implemented clustering strategies are:
		- Planar segmentation.
		- Euclidean cluster extraction.
		- Region growing segmentation.

3. Descriptor Generator
	The spin image of a down-sample of the cluster is calculated. Downsampling is performed via the voxel-grid downsampler.
	
4. Correspondence Matching
	For each category, every spin image of every cluster is matched using KdTree to the best spin image of every model. 
	Then the cross-correlation coefficient is calculated and if it is larger than a threshold a match is detected. 
	If the percentage of matches of the spin images of the scene exceeds a threshold then there is a match. 

5. Future Work
	If more time was given the training module should return the principal components of each category. 
	Recognition should be done considering the projection of each scene into this principal components. 
	