/*
 * parser_sequencer.cpp
 *
 *	Sequencer for parsing.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#include "io/parser_sequencer.h"

// Constructor for parser sequencer.
ParserSequencer::ParserSequencer ()
{
}

// Destructor for parser sequencer.
ParserSequencer::~ParserSequencer ()
{
}

// Main parser function.
bool
ParserSequencer::parseAll (ParametersBag& params_bag_output)
{
  // Bags of parameters.
  TopicsParameters topics_params;
  LocalizerParameters localizer_params;
  std::string node;

  // Error flag.
  bool error;

  // Parse parameters.
  if (parseTopics (topics_params))
  {
  	error = true;
  }
  else if (parseLocalizer (localizer_params))
  {
  	error = true;
  }

  // Store parameters.
  params_bag_output.topics = topics_params;
  params_bag_output.localizer = localizer_params;

  // Return error.
  return error;
}

// Find files in directories.
void
ParserSequencer::findFiles( const boost::filesystem::path & dir_path, std::vector<boost::filesystem::path> & paths_found) {
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(dir_path); itr != end_itr; ++itr)
    {
      if (boost::filesystem::is_directory(itr->status()))
      {
	  findFiles (itr->path(), paths_found);
      }
      else if (itr->path().extension() == ".pcd") // see below
      {
	    paths_found.push_back(itr->path());
      }
    }
}

// Parse topics parameters
bool
parseTopics (TopicsParameters& topics_params_output)
{
  TopicsParameters topics_params;

  // Error flag.
  bool error;

  // Node handle.
  ros::NodeHandle nh;

  // Parse parameters.
  if (!nh.getParam("topics/odometry", topics_params.odometry))
  {
    std::cerr << "Odometry topic name is missing, no input available.\n";
    error = true;
  }
  else if (!nh.getParam("topics/sensor", topics_params.sensor))
  {
    std::cerr << "Sensor topic name is missing, no input available.\n";
    error = true;
  }
  else if (!nh.getParam("topics/map", topics_params.map))
  {
  	std::cerr << "Map topic name is missing, no input available.\n";
    error = true;
  }
  else if (!nh.getParam("topics/tf", topics_params.tf))
  {
    std::cerr << "Tf topic name is missing, no input available.\n";
    error = true;
  }
  else if (!nh.getParam("topics/onOff", topics_params.onOffSubscriber))
	{
		std::cerr << "OnOff topic name is missing, no input available.\n";
		error = true;
	}
  else
  {
    std::cerr << "Topics parameters parsed.\n";
  }

  nh.param<std::string>("topics/publisher", topics_params.publisher, "localization/odometry");
  nh.param<std::string>("topics/flag", topics_params.flag, "localization/is_localized");
  nh.param<std::string>("topics/visualizer", topics_params.visualizer, "localization/visualizer");

  // Store parameters.
  topics_params_output = topics_params;

  // Return error.
  return error;

}

// Parse localizer parameters.
bool
parseLocalizer (LocalizerParameters& localizer_params_output)
{
	// Parameters
  LocalizerParameters localizer_params;
  std::string algorithm;

  // Error flag.
  bool error;

  // Node handle.
  ros::NodeHandle nh;

  // Parse parameters.
  if (!nh.getParam("localizer/algorithm/type", algorithm))
  {
    std::cerr << "Algorithm name is missing, no input available.\n";
    error = true;
  }

  // Parse algorithm specific parameters.
  if (!error)
  {
		if (algorithm == "Particle_filter")
		{
			// Parse particle filter parameters.
			ParticleFilterParameters particle_filter_params;
			error = parseParticleFilterParams(particle_filter_params);

			// Parse frame_id for visualizer.
			std::string frame_id;
			nh.param<std::string>("visualizer/frame_id", frame_id, "odometry_link");

			// Create publisher, visualizer and algorithm.
			if (!error)
			{
				localizer_params.pose_publisher = boost::make_shared<LocalizationPublisher>(frame_id);
				localizer_params.visualizer = boost::make_shared<ParticlesVisualizer>(frame_id);
				localizer_params.algorithm = boost::make_shared<ParticleFilter>(particle_filter_params);
			}
		}
		else
		{
			std::cout << "No recognised algorithm name.\n";
			error = true;
		}
  }

  // Store parameters.
  localizer_params_output = localizer_params;

  // Return error.
  return error;
}

// Parse particle filter parameters.
bool
parseParticleFilterParams(ParticleFilterParameters& particle_filter_params_output)
{
	// Parameters.
	ParticleFilterParameters particle_filter_params;
	std::string motion_updater;
	std::string sensor_updater;
	std::string resampler;
	int cells_jumped;

	// Node handle.
	ros::NodeHandle nh;

	// Error flag.
	bool error;

  // Parse functions to be used in the algorithm.
  // Parse motion updater to be used.
  if (!nh.getParam("localizer/motion_updater/type", motion_updater))
	{
		std::cerr << "Motion updater name is missing, no input available.\n";
		error = true;
	}

  // Parse sensor updater to be used.
	if (!nh.getParam("localizer/sensor_updater/type", sensor_updater))
	{
		std::cerr << "Sensor updater name is missing, no input available.\n";
		error = true;
	}

	// Parse resampler type to be used.
	if (!nh.getParam("localizer/resampler/type", resampler))
	{
		std::cerr << "Sampler name is missing, no input available.\n";
		error = true;
	}

  // Parse other needed parameters.
	double val;
  nh.param<int>("localizer/others/cells_jumped", particle_filter_params.cells_jumped, 1); // Parse cells_jumped parameter.
  nh.param<double>("localizer/others/delta_theta", val, 0.15);// Parse delta_theta parameter.

  particle_filter_params.delta_theta = val;
  nh.param<double>("localizer/others/position_threshold", val, 1.0); // Parse position_threshold parameter.
  particle_filter_params.position_threshold = val;
  nh.param<double>("localizer/others/angle_threshold", val, 1.0); // Parse angle threshold parameter.
  particle_filter_params.angle_threshold = val;
  // Resolve type of motion updater and create an instance.
  if (motion_updater == "Sample_odometry_motion_model")
	{
  		// Parse sampler parameters.
  		SamplerParameters sampler;
  		error = parseSamplerParams (sampler);

  		// Parse odometry motion model parameters.
  		OdometryMotionModelParams odo_params;
  		error = parseOdometryMotionModelParams (odo_params);

  		// Create instance of odometry motion updater.
  		if (!error)
			{
				particle_filter_params.motion_updater = boost::make_shared<OdometryMotionModel>(odo_params, sampler.distribution) ;
			}
	}
	else
	{
		error = true;
	}

  // Resolve type of sensor updater and create an instance.
	if (sensor_updater == "Map_matching")
	{
			// Parse map matching parameters.
			MapMatchingParams map_matching_params;
			error = parseMapMatchingParams (map_matching_params);

			// Create instance of sensor updater.
			if (!error)
			{
				particle_filter_params.sensor_updater = boost::make_shared<MapMatching>(map_matching_params);
			}
	}
	else
	{
		error = true;
	}

	// Resolve type of resample and create an instance.
	if (resampler == "Roulette_wheel_sampler")
	{
			// Parse roulette wheel sampler parameters.
			RWSParams rws_params;
			error = parseRWSParams (rws_params);

			// Create instance of roulette wheel sampler.
			if (!error)
			{
				particle_filter_params.resampler = boost::make_shared<RWS>(rws_params);
			}
	}
	else
	{
		error = true;
	}

	// Store parameters.
	particle_filter_params_output = particle_filter_params;

	// Return error.
	return error;
}

// Parse sampler parameters.
bool
parseSamplerParams(SamplerParameters& sampler_params_output)
{
	// Parameters
	SamplerParameters sampler_params;
	std::string distribution;

	// Error flag.
	bool error;

	// Node handle.
	ros::NodeHandle nh;

	// Parse parameters.
	if (!nh.getParam("localizer/motion_updater/distribution", distribution))
	{
		std::cerr << "Distribution name for sampler is missing, no input available.\n";
		error = true;
	}

	// Inspect parameters.
	if (distribution == "Triangular_distribution")
	{
		// Create instance of triangle distribution sampler.
		sampler_params.distribution = boost::make_shared<TriangularDistribution>();
	}
	else if (distribution == "Approximate_normal_distribution")
	{
			// Create instance of approximate normal distribution.
		sampler_params.distribution = boost::make_shared<ApproxNormalDistribution>();
	}
	else
	{
		std::cerr << "Distribution name not recognized.\n";
		error = true;
	}

	// Store parameters.
	sampler_params_output = sampler_params;

	// Return error.
	return error;
}

// Parse odometry motion model params.
bool
parseOdometryMotionModelParams(OdometryMotionModelParams& odo_params_output)
{
	// Parameters
	OdometryMotionModelParams odo_params;
	double a1, a2, a3, a4;

	// Error flag.
	bool error;

	// Node handle.
	ros::NodeHandle nh;

	// Parse parameters.
	if (!nh.getParam("localizer/motion_updater/a1", a1))
	{
		std::cout << "a1 parameter for odometry model missing, no input available.\n";
		error = true;
	}
	else if (!nh.getParam("localizer/motion_updater/a2", a2))
	{
		std::cout << "a2 parameter for odometry model missing, no input available.\n";
		error = true;
	}
	else if (!nh.getParam("localizer/motion_updater/a3", a3))
	{
		std::cout << "a3 parameter for odometry model missing, no input available.\n";
		error = true;
	}
	else if (!nh.getParam("localizer/motion_updater/a4", a4))
	{
		std::cout << "a4 parameter for odometry model missing, no input available.\n";
		error = true;
	}

	odo_params.a1 = a1;
	odo_params.a2 = a2;
	odo_params.a3 = a3;
	odo_params.a4 = a4;

	// Store parameters.
	odo_params_output = odo_params;

	// Return error.
	return error;
}

// Parse map matching parameters.
bool
parseMapMatchingParams (MapMatchingParams& sampler_params_output)
{
	// Parameters.
	MapMatchingParams sampler_params;

	// Error flag.
	bool error;

	// Node Handle.
	ros::NodeHandle nh;

	// Store parameters.
	sampler_params_output = sampler_params;

	// Return error.
	return error;
}

// Parse Roulette wheel parameters.
bool
parseRWSParams (RWSParams& resampler_params_output)
{
	//Parameters.
	RWSParams resampler_params;

	// Error flag.
	bool error;

	// Node handle.
	ros::NodeHandle nh;

	// Store parameters.
	resampler_params_output = resampler_params;

	// Return error.
	return error;
}
