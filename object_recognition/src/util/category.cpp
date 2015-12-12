/*
 * category.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/util/category.h"

static const std::string NAME = "UNKNOWN";
Category::Category()
{
  std::string name = NAME;
  _name = name;
  _is_detected = false;
}

Category::Category(std::string name){
  _name = name;
  _is_detected = false;
}

void Category::add_learning_set(const DescriptorCloud & learning_set){
  _learning_set.push_back(learning_set);
}

void Category::add_learning_set(const std::string & path){
  DescriptorCloud learning_set;
  if (pcl::io::loadPCDFile<DescriptorType >(path, learning_set) == -1){
      ROS_ERROR("file %s not found", path.c_str());
  }
  _learning_set.push_back(learning_set);
}

void Category::set_detected(bool is_detected){
  _is_detected = is_detected;
}


void Category::set_color(double r, double g, double b){
  _color.r = r;
  _color.g = g;
  _color.b = b;
}

void Category::set_color(struct Color color){
  _color = color;
}

struct Color Category::get_color(void){
  return _color;
}


void Category::set_name(std::string name){
  _name = name;
}

bool Category::is_detected(void){
  return _is_detected;
}

const std::string & Category::get_name(void){
  return _name;
}

std::vector<DescriptorCloud> Category::get_learning_set(void){
  return _learning_set;
}
