/*
 * localizer_parameters.h
 *
 *	Localizer parameters.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_PARAMETERS_LOCALIZER_PARAMETERS_H_
#define LOCALIZATION_INCLUDE_PARAMETERS_LOCALIZER_PARAMETERS_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <algorithms/motion_updater/motion_updater.h>
#include <algorithms/resampler/resampler.h>
#include <algorithms/sensor_updater/sensor_updater.h>
#include <probability_distributions/probability_distributions.h>
#include <ros/publisher.h>

// Forward declaration of virtual classes.
class LocalizationAlgorithm; ///< Localization generic algorithm.
class Visualizer; ///< Visualizer generic class.
class LocalizationPublisher; ///< Publisher of the localization pose generic class.

// Algorithm parameters.
class AlgorithmParameters {
public:

	// Virtual destructor.
	virtual ~AlgorithmParameters ();

	// Retrieve params.
	virtual std::vector<std::string> getParams() = 0;
};

class ParticleFilterParameters: public AlgorithmParameters
{
public:

	// Virtual destructor.
	virtual ~ParticleFilterParameters ();

	// Retrieve params.
	std::vector<std::string> getParams();

	// Algorithms to be used.
	boost::shared_ptr<MotionUpdater> motion_updater; ///< Motion updater generic class.
	boost::shared_ptr<SensorUpdater> sensor_updater; ///< Sensor updater generic class.
	boost::shared_ptr<Resampler> resampler; ///< Resampler generic class.

	// Other parameters.
	int cells_jumped; ///< Number of cells to jump when creating the particles set.
	float delta_theta; ///< Angle step between angles of the initial particles.
	float angle_threshold; ///< Angle threshold for deciding whether we are localized or not.
	float position_threshold; ///< Position threshold for deciding whether we are localized or not.
};

// Parameters for general virtual class of localizers.
struct LocalizerParameters
{
	// Algorithm for localization.
	boost::shared_ptr<LocalizationAlgorithm> algorithm;

	// Set of algorithm parameters.
  boost::shared_ptr<AlgorithmParameters> algo_params;

  // Visualizer for particles publisher.
	boost::shared_ptr<LocalizationPublisher> pose_publisher;

	// Flag publisher for knowing if we are localized.
	ros::Publisher flag_publisher;

  // Visualizer for particles publisher.
  boost::shared_ptr<Visualizer> visualizer;
};

// Parameters for sampler class.
struct SamplerParameters
{
	// Generic probability distribution.
	boost::shared_ptr<ProbabilityDistribution> distribution;
};

#endif /* LOCALIZATION_INCLUDE_PARAMETERS_LOCALIZER_PARAMETERS_H_ */
