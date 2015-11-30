/** @file visualization_marker.h
 *  @brief Class for generating and storing ROS Markers.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <visualization_msgs/Marker.h>
#include <string.h>
#include <ros/ros.h>
#include "../ros/topics.h"
#ifndef _MARKER_MSG
#define _MARKER_MSG

/**
 * @brief A data structure for handling the color of markers.
 */
struct Color
{
  double r; ///< red intensity
  double g; ///< green intensity
  double b; ///< blue intensity
  Color(): r(0.0), g(0.0), b(0.0) {}; ///< Default constructor
  Color(double rr, double gg, double bb): r(rr), g(gg), b(bb) {}; ///< Struct constructor
};

/**
 * @brief Marker Msg container.
 */
class MarkerMessage
{
private:
  visualization_msgs::Marker _marker; ///< the actual msg.

public:

  MarkerMessage(); ///< Default constructor.
  void update_timestamp(void); ///< Update msg timestamp.

  /**
   * Set msg frame id.
   * @param frame
   */
  void set_frame_id(const std::string & frame);

  /**
   * Set msg namespace.
   * @param ns
   */
  void set_namespace(const std::string & ns);

  /**
   * Set unique id.
   * @param id
   */
  void set_id(unsigned int id);

  /**
   * Set marker type.
   * @param type
   */
  void set_type(unsigned int type);

  /**
   * Set marker action (ADD, DELETE, REPLACE).
   * @param action
   */
  void set_action(unsigned int action);

  /**
   * Set marker mass center position.
   * @param x x coordinate
   * @param y y coordinate
   * @param z z coordinate
   */
  void set_position(double x, double y, double z);

  /**
   * Set orientation in quaternion form.
   * @param x x vector component of quaternion
   * @param y y vector component of quaternion
   * @param z z vector component of quaternion
   * @param w scalar component of quaternion.
   */
  void set_orientation(double x, double y, double z, double w);

  /**
   * Set scale of marker.
   * @param x scale of marker
   */
  void set_scale(double x);

  /**
   * Set scale of marker.
   * @param x width
   * @param y height
   * @param z depth
   */
  void set_scale(double x, double y, double z);

  /**
   * Set marker color.
   * @param r red intensity
   * @param g green intensity
   * @param b blue intensity
   */
  void set_color(double r, double g, double b);

  /**
   * Set marker color
   * @param color filled color struct
   */
  void set_color(struct Color color);
  void set_green(void); ///< Set marker green.
  void set_blue(void); ///< Set marker blue.
  void set_red(void); ///< Set marker red.

  /**
   * Set marker name
   * @param name
   */
  void set_text(const std::string & name);

  /**
   * Set to show or not
   * @param s when `true' it will add.
   */
  void show(bool s);

  /**
   * Set transparency of marker
   * @param a transparency value
   */
  void set_transparency(double a);

  /**
   * Set timestamp
   * @param timestamp
   */
  void set_timestamp(ros::Time timestamp);
  /**
   * Get the marker msg.
   * @return marker msg
   */
  visualization_msgs::Marker get_marker(void);
};

#endif //_MARKER_MSG
