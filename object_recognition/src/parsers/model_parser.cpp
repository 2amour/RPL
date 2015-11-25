/*
 * model_parser.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Sebastian Curi
 */

#include "model_parser.h"

std::vector<Category> get_models(const std::string & file_name)
{
  std::vector<Category> models;
  std::vector<FilePair> classes;
  classes = get_file_names(file_name);

  Category ducks(DUCK_CLASS);
  Category humans(HUMAN_CLASS);
  Category unknown(UNKNOWN_CLASS);

  Color duck_color(0.0, 0.0, 1.0);
  Color human_color(0.0, 1.0, 0.0);
  Color unknown_color(1.0, 0.0, 0.0);

  ducks.set_color(duck_color);
  humans.set_color(human_color);
  unknown.set_color(unknown_color);

  for (std::vector<FilePair>::iterator it = classes.begin(); it != classes.end(); ++it)
  {
    if (it->first == DUCK_CLASS)
    {
      ducks.add_learning_set(it->second);
    }
    else if (it->first == HUMAN_CLASS)
    {
      humans.add_learning_set(it->second);
    }
    else
    {
      ROS_WARN("Not a known class");
    }
  }

  models.push_back(ducks);
  models.push_back(humans);
  models.push_back(unknown);

  return models;
}
