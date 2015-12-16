/**
 * @file sampling_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs.
 */
#include <ros/ros.h>
#include "localization/sampling_strategies/roulette_wheel_sampling.h"
#include "localization/sampling_strategies/stochastic_universal_sampling.h"

#ifndef SAMPLING_PARSER_H_
#define SAMPLING_PARSER_H_

/**
 * @brief Class that parses from the rosparam server the
 * sampling strategy to be used.
 */
class SamplingParser
{
public:
  SamplingParser(ros::NodeHandle nh); ///< Default Constructor
  ~SamplingParser(); ///< Class Deconstructor
  SamplingPtr get_sampling_strategy(); ///< Get parsed sampling strategy implementation.
private:
  SamplingPtr _sampling_strategy; ///< Parsed sampling strategy implementation.
};

#endif /* SAMPLING_PARSER_H_ */
