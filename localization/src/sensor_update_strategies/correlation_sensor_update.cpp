/**
 * @file correlation_sensor_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sensor_update_strategies/correlation_sensor_update.h"

static const std::string NAME = "Correlation Sensor Model";

CorrelationSensorUpdate::CorrelationSensorUpdate()
{
  _name = NAME;
}

double CorrelationSensorUpdate::a_posteriori(double a_priori, OccupancyMapPtr a_priori_map, geometry_msgs::PoseWithCovarianceStamped pose,
                                             const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double theta = 2 * std::atan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);

  for (int index = 0; index < msg->ranges.size(); ++index)
  {
    if (std::isnan(msg->ranges[index]) | std::abs(msg->ranges[index]) > msg->range_max)
    {
      continue;
    }
    double delta_theta = index * msg->angle_increment + msg->angle_min;
    double xk = x + msg->ranges[index] * std::cos(theta + delta_theta);
    double yk = y + msg->ranges[index] * std::sin(theta + delta_theta);
    if (!(xk > _map.get_x_max() + 2*_map.get_resolution() or
          xk < _map.get_x_min() - 2*_map.get_resolution() or
          yk > _map.get_y_max() + 2*_map.get_resolution() or
          yk < _map.get_y_min() - 2*_map.get_resolution()))
    {
      a_priori_map->set_value(std::make_pair(xk, yk), 100);
    }else{
      //return 0;
    }
  }
  return _map.correlation(*a_priori_map.get());
}

