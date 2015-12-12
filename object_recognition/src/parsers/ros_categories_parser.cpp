/**
 * @file ros_categories_parser.cpp
 *
 *  @date Dec 11, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "object_recognition/parsers/ros_categories_parser.h"

static const std::string CATEGORIES_KEY = "categories";
static const std::string NAMES_KEY = "names";

static const std::string PATH_KEY = "path";
static const std::string COLOR_KEY = "color";

RosCategoriesParser::RosCategoriesParser(ros::NodeHandle nh){
  std::vector<std::string> categories;
  std::vector<std::string> learning_set_path;
  std::vector<float> rgb;

  nh.getParam("/" + CATEGORIES_KEY + "/" + NAMES_KEY, categories);

  for (std::vector<std::string>::iterator it = categories.begin(); it < categories.end(); ++it){
    Category new_category(*it);
    nh.getParam("/" + CATEGORIES_KEY + "/" + *it + "/" + COLOR_KEY, rgb);
    new_category.set_color(rgb[0], rgb[1], rgb[2]);
    if (nh.hasParam("/" + CATEGORIES_KEY + "/" + *it + "/" + PATH_KEY)){
      nh.getParam("/" + CATEGORIES_KEY + "/" + *it + "/" + PATH_KEY, learning_set_path);
      for (std::vector<std::string>::iterator iit = learning_set_path.begin(); iit < learning_set_path.end(); ++iit){
        new_category.add_learning_set(*iit);
      }
    }
    _categories.push_back(new_category);
  }
}


std::vector<Category> RosCategoriesParser::get_categories(){
  return _categories;
}
