/**
 * @file tf_parameter_parser.h
 *
 * Class that parses from a ROS parameter server the parameters of a given frame.
 */

#include <tf/transform_broadcaster.h>
#include <string>

#ifndef TF_PARAMETER_PARSER_H_
#define TF_PARAMETER_PARSER_H_

static const std::string PARENT_PARAM_KEY = "parent";
static const std::string ORIGIN_PARAM_KEY = "origin";
static const std::string ROTATION_PARAM_KEY = "rotation";
static const std::string X_PARAM_KEY = "x";
static const std::string Y_PARAM_KEY = "y";
static const std::string Z_PARAM_KEY = "z";
static const std::string W_PARAM_KEY = "w";
static const std::string YAW_PARAM_KEY = "yaw";
static const std::string PITCH_PARAM_KEY = "pitch";
static const std::string ROLL_PARAM_KEY = "roll";

/**
 * @brief Class that parses tf parameters from ros server
 */
class TfParameterParser{
private:
  tf::Transform _transform; ///< Parsed transform
  std::string _parent; ///< String with parent frame id
  std::string _child; ///< String with child frame id
  void parse_parent(ros::NodeHandle nh, std::string child_frame_id); ///< helper to parse parents name
  void parse_origin(ros::NodeHandle nh, std::string child_frame_id); ///< helper to parse childs origin
  void parse_rotation(ros::NodeHandle nh, std::string child_frame_id); ///< helper to parse childs rotation
public:

  /**
   * Class constructor
   * @param nh ros node handler
   * @param child_frame_id frame of child to parse
   */
  TfParameterParser(ros::NodeHandle nh, std::string child_frame_id);

  /**
   * Get the stamped message to broadcast
   * @return
   */
  tf::StampedTransform get_stamped_transform(void);

  /**
   * Get the transform
   * @return
   */
  tf::Transform get_transform(void);

  /**
   * Get the pearent id
   * @return
   */
  std::string get_parent(void);
};

#endif /* TF_PARAMETER_PARSER_H_ */
