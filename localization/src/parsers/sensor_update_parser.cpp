/**
 * @file sensor_update_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/parsers/sensor_update_parser.h"

static const std::string SENSOR_UPDATE_KEY = "sensor_update";
static const std::string NAME_KEY = "name";
static const std::string CORRELATION_KEY = "correlation_model";
static const std::string LIKELIHOOD_KEY = "likelihood_model";
static const std::string Z_RAND_KEY = "z_rand";
static const std::string Z_VARIANCE_KEY = "z_var";

SensorUpdateParser::SensorUpdateParser(ros::NodeHandle nh){
  std::string name;
  _sensor_update.reset(new CorrelationSensorUpdate());

  if (nh.hasParam(SENSOR_UPDATE_KEY)){
    nh.getParam("/" + SENSOR_UPDATE_KEY + "/" + NAME_KEY, name);
    if (name == CORRELATION_KEY){
      _sensor_update.reset(new CorrelationSensorUpdate());
      ROS_INFO("Set correlation sensor model");
    }else if (name == LIKELIHOOD_KEY){
      double z_rand, z_var;
      nh.getParam("/" + SENSOR_UPDATE_KEY + "/" + Z_RAND_KEY, z_rand);
      nh.getParam("/" + SENSOR_UPDATE_KEY + "/" + Z_VARIANCE_KEY, z_var);
      _sensor_update.reset(new LikelihoodSensorUpdate(z_rand, z_var));
      ROS_INFO("Set likelihood sensor model");
    }else{
      ROS_WARN("Sensor model not recognized. Set correlation sensor model by default");
    }
  }else{
    ROS_WARN("No sensor model specified. Set correlation sensor model by default");
  }
}

SensorUpdatePtr SensorUpdateParser::get_sensor_update(){
  return _sensor_update;
}

SensorUpdateParser::~SensorUpdateParser(){
  _sensor_update.reset();
}
