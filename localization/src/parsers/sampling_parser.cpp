/**
 * @file sampling_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/parsers/sampling_parser.h"

static const std::string SAMPLING_KEY = "sampling_strategy";
static const std::string NAME_KEY = "name";
static const std::string ROULETTE_KEY = "roulette";
static const std::string STOCHASTIC_KEY = "stochastic";



SamplingParser::SamplingParser(ros::NodeHandle nh){
  std::string name;
  _sampling_strategy.reset(new StochasticUniversalSampling());

  if (nh.hasParam(SAMPLING_KEY)){
    nh.getParam("/" + SAMPLING_KEY + "/" + NAME_KEY, name);
    if (name == ROULETTE_KEY){
      _sampling_strategy.reset(new RouletteWheelSampling());
      ROS_INFO("Set roulette wheel sampling");
    }else if (name == STOCHASTIC_KEY){
      _sampling_strategy.reset(new StochasticUniversalSampling());
      ROS_INFO("Set stochastic universal sampling");
    }else{
      ROS_WARN("Sampling strategy not recognized. Set stochastic universal sampling by default.");
    }
  }else{
    ROS_WARN("No sampling strategy specified. Set stochastic universal sampling by default.");
  }
}

SamplingPtr SamplingParser::get_sampling_strategy(){
  return _sampling_strategy;
}

SamplingParser::~SamplingParser(){
  _sampling_strategy.reset();
}
