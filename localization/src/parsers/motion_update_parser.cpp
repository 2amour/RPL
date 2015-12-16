/**
 * @file motion_update_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/parsers/motion_update_parser.h"

static const std::string MOTION_UPDATE_KEY = "motion_update";
static const std::string NAME_KEY = "name";
static const std::string ODOMETRY_KEY = "odometry_model";
static const std::string VELOCITY_KEY = "velocity_model";
static const std::string VARIANCE_KEY = "variance";

MotionUpdateParser::MotionUpdateParser(ros::NodeHandle nh){
  std::string name;
  std::vector<double> variance;
  _motion_update.reset(new OdometryMotionUpdate());

  if (nh.hasParam(MOTION_UPDATE_KEY)){
    nh.getParam("/" + MOTION_UPDATE_KEY + "/" + NAME_KEY, name);
    nh.getParam("/" + MOTION_UPDATE_KEY + "/" + VARIANCE_KEY, variance);
    if (name == ODOMETRY_KEY){
       _motion_update.reset(new OdometryMotionUpdate());
       ROS_INFO("Set odometry model");
    }else if (name == VELOCITY_KEY){
      _motion_update.reset(new VelocityMotionUpdate());
      ROS_INFO("Set velocity model");
    }else{
      ROS_WARN("Motion model not recognized. Set odometry motion model by default");
    }
  }else{
    ROS_WARN("No motion model specified. Set odometry motion model by default");
  }
  _motion_update->set_variance(variance);
}

MotionUpdatePtr MotionUpdateParser::get_motion_update(){
  return _motion_update;
}

MotionUpdateParser::~MotionUpdateParser(){
  _motion_update.reset();
}
