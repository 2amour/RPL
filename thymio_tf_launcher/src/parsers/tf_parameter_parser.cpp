/*
 * tf_parameter_parser.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: Sebastian Curi
 */

#include "tf_parameter_parser.h"

TfParameterParser::TfParameterParser(ros::NodeHandle nh, std::string child_frame_id){
  _child = child_frame_id;
  if (nh.hasParam("/"+child_frame_id)){
    parse_parent(nh, _child);
    parse_origin(nh, _child);
    parse_rotation(nh, _child);
  }else{
    ROS_ERROR("No parameters for frame %s", _child.c_str());
  }

}

void TfParameterParser::parse_parent(ros::NodeHandle nh, std::string child_frame_id){
  nh.getParam("/" + child_frame_id + "/" + PARENT_PARAM_KEY, _parent);
}

void TfParameterParser::parse_origin(ros::NodeHandle nh, std::string child_frame_id){
  double x, y, z;
  nh.getParam("/" + child_frame_id + "/" + ORIGIN_PARAM_KEY + "/" + X_PARAM_KEY, x);
  nh.getParam("/" + child_frame_id + "/" + ORIGIN_PARAM_KEY + "/" + Y_PARAM_KEY, y);
  nh.getParam("/" + child_frame_id + "/" + ORIGIN_PARAM_KEY + "/" + Z_PARAM_KEY, z);
  _transform.setOrigin(tf::Vector3(x, y, z));
}

void TfParameterParser::parse_rotation(ros::NodeHandle nh, std::string child_frame_id){
  tf::Quaternion q;
  if (nh.hasParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + YAW_PARAM_KEY)){
    double yaw, pitch, roll;
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + YAW_PARAM_KEY, yaw);
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + PITCH_PARAM_KEY, pitch);
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + ROLL_PARAM_KEY, roll);
    q.setRPY(roll, pitch, yaw);
  }else{
    double x, y, z, w;
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + X_PARAM_KEY, x);
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + Y_PARAM_KEY, y);
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + Z_PARAM_KEY, z);
    nh.getParam("/" + child_frame_id + "/" + ROTATION_PARAM_KEY + "/" + W_PARAM_KEY, w);
    q.setW(w); q.setX(x); q.setY(y); q.setZ(z);
  }
  _transform.setRotation(q);
}

tf::StampedTransform TfParameterParser::get_stamped_transform(void){
  tf::StampedTransform tf_stamped(_transform, ros::Time::now(), _parent, _child);
  return tf_stamped;
}


tf::Transform TfParameterParser::get_transform(void){
  return _transform;
}

std::string TfParameterParser::get_parent(void){
  return _parent;
}
