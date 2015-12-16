/*
 * parser_sequencer.h
 *
 *	Sequence the parsing procedure
 *  Created on: Dec 4, 2015
 *      Author: toni
 */

#ifndef LOCALIZATION_INCLUDE_IO_PARSER_SEQUENCER_H_
#define LOCALIZATION_INCLUDE_IO_PARSER_SEQUENCER_H_

#include <ros/ros.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"

#include "parameters/parameters_bag.h"
#include "algorithms/localizer/localization_algorithm.h"
#include "visualizer/visualizer.h"

// Sequencer for parsing parameters.
class ParserSequencer
{
	public:

		// Constructor of parser sequencer.
		ParserSequencer ();

		// Destructor of parser sequencer.
		~ParserSequencer ();

		// Main parameters parser.
		bool parseAll (ParametersBag& params_bag_output);

	private:

		// Find files in directories.
		void findFiles( const boost::filesystem::path & dir_path, std::vector<boost::filesystem::path> & paths_found);
};

// Parse topics parameters.
bool parseTopics (TopicsParameters& topics_params_output);

// Parse localizer parameters.
bool parseLocalizer (LocalizerParameters& localizer_params_output);

// Parse particle filter parameters.
bool parseParticleFilterParams(ParticleFilterParameters& particle_filter_params_output);

// Parse sampler parameters.
bool parseSamplerParams(SamplerParameters& sampler_params_output);

// Parse odometry motion model parameters.
bool parseOdometryMotionModelParams(OdometryMotionModelParams& odo_params_output);

// Parse map matching params.
bool parseMapMatchingParams (MapMatchingParams& map_params_output);

// Parse roulette wheel sampler parameters;
bool parseRWSParams (RWSParams& resampler_params_output);

#endif /* LOCALIZATION_INCLUDE_IO_PARSER_SEQUENCER_H_ */
