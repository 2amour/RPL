#include "object_recognition/msg/visualization_marker.h"

MarkerMessage::MarkerMessage()
{
  _marker = visualization_msgs::Marker();
  _marker.header.frame_id = "base_link";
  _marker.header.stamp = ros::Time();
  _marker.ns = "spin_recognition";
  _marker.id = 0;
  _marker.type = visualization_msgs::Marker::CUBE;
  _marker.action = visualization_msgs::Marker::ADD;
  _marker.pose.position.x = 1;
  _marker.pose.position.y = 1;
  _marker.pose.position.z = 1;
  _marker.pose.orientation.x = 0.0;
  _marker.pose.orientation.y = 0.0;
  _marker.pose.orientation.z = 0.0;
  _marker.pose.orientation.w = 1.0;
  _marker.scale.x = 1;
  _marker.scale.y = 0.1;
  _marker.scale.z = 0.1;
  _marker.color.a = 0.4; // Don't forget to set the alpha!
  _marker.color.r = 0.0;
  _marker.color.g = 1.0;
  _marker.color.b = 0.0;
}

void MarkerMessage::update_timestamp(void)
{
  _marker.header.stamp = ros::Time();
}
void MarkerMessage::set_timestamp(ros::Time timestamp)
{
  _marker.header.stamp = timestamp;
}
void MarkerMessage::set_frame_id(const std::string & frame)
{
  _marker.header.frame_id = frame.c_str();
}
void MarkerMessage::set_namespace(const std::string & ns)
{
  _marker.ns = ns;
}

void MarkerMessage::set_id(unsigned int id)
{
  _marker.id = id;
}

void MarkerMessage::set_type(unsigned int type)
{
  _marker.type = type;
}
void MarkerMessage::set_action(unsigned int action)
{
  _marker.action = action;
}
void MarkerMessage::set_position(double x, double y, double z)
{
  _marker.pose.position.x = x;
  _marker.pose.position.y = y;
  _marker.pose.position.z = z;
}
void MarkerMessage::set_orientation(double x, double y, double z, double w)
{
  _marker.pose.orientation.x = x;
  _marker.pose.orientation.y = y;
  _marker.pose.orientation.z = z;
  _marker.pose.orientation.w = w;
}
void MarkerMessage::set_scale(double x)
{
  _marker.scale.x = x;
  _marker.scale.y = x;
  _marker.scale.z = x;
}
void MarkerMessage::set_scale(double x, double y, double z)
{
  _marker.scale.x = x;
  _marker.scale.y = y;
  _marker.scale.z = z;
}
void MarkerMessage::set_color(double r, double g, double b)
{
  _marker.color.r = r;
  _marker.color.g = g;
  _marker.color.b = b;
}

void MarkerMessage::set_color(struct Color color)
{
  _marker.color.r = color.r;
  _marker.color.g = color.g;
  _marker.color.b = color.b;
}


void MarkerMessage::set_red(void)
{
  set_color(1, 0, 0);
}
void MarkerMessage::set_green(void)
{
  set_color(0, 1, 0);
}
void MarkerMessage::set_blue(void)
{
  set_color(0, 0, 1);
}

void MarkerMessage::set_transparency(double a)
{
  _marker.color.a = a;
}

void MarkerMessage::set_text(const std::string & name)
{
  _marker.text = name.c_str();
}

void MarkerMessage::show(bool s)
{
  s ? set_action(visualization_msgs::Marker::ADD) : set_action(visualization_msgs::Marker::DELETE);
}

visualization_msgs::Marker MarkerMessage::get_marker(void){
  //update_timestamp();
  return _marker;
}
