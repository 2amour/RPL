/**
 * @file filter_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/parsers/filter_parser.h"

static const std::string FILTER_KEY = "filter";
static const std::string NAME_KEY = "name";
static const std::string PARTICLE_FILTER_KEY = "particle_filter";
static const std::string NUMBER_OF_PARTICLES_KEY = "particles";
static const std::string RANDOM_SAMPLES_KEY = "random_particles";
static const std::string WEIGHT_THRESHOLD_KEY = "weight_threshold";



FilterParser::FilterParser(ros::NodeHandle nh)
{
  std::string name;
  int n_particles = 100;
  int random_samples = 0;
  double weight_threshold = 0.5;
  _filter.reset(new ParticleFilter());

  if (nh.hasParam("/" + FILTER_KEY)){
    nh.getParam("/" + FILTER_KEY + "/" + NAME_KEY, name);
    nh.getParam("/" + FILTER_KEY + "/" + WEIGHT_THRESHOLD_KEY, weight_threshold);
    nh.getParam("/" + FILTER_KEY + "/" + NUMBER_OF_PARTICLES_KEY, n_particles);
    nh.getParam("/" + FILTER_KEY + "/" + RANDOM_SAMPLES_KEY, random_samples);


    if (name == PARTICLE_FILTER_KEY){
      _filter.reset(new ParticleFilter(n_particles, random_samples, weight_threshold));
      ROS_INFO("Set particle filter with %d particles", n_particles);
    }else{
      ROS_WARN("Filter not recognized. Set particle filter with %d by default", n_particles);
    }
  }else{
    ROS_WARN("Filter not specified. Set particle filter with %d by default", n_particles);
  }
}

PoseFilterPtr FilterParser::get_filter()
{
  return _filter;
}

FilterParser::~FilterParser()
{
  _filter.reset();
}
