/**
 * @file likelihood_sensor_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sensor_update_strategies/likelihood_sensor_update.h"

static const std::string NAME = "Likelihood Sensor Model";

LikelihoodSensorUpdate::LikelihoodSensorUpdate()
{
  _name = NAME;
  _z_rand = 0.0;
  _z_var = 0.0;
}

LikelihoodSensorUpdate::LikelihoodSensorUpdate(double z_rand, double z_var)
{
  _name = NAME;
  _z_rand = z_rand;
  _z_var = z_var;
}

double LikelihoodSensorUpdate::a_posteriori(double a_priori, OccupancyMapPtr a_priori_map, geometry_msgs::PoseWithCovarianceStamped pose,
                                            const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double probability = 0.0;
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double theta = 2 * std::atan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  double resolution = _map.get_resolution() / _z_var;

  std::pair<double, double> last_point = std::make_pair(0,0);
  int count = 0;
  for (int index = 0; index < msg->ranges.size(); ++index)
  {
    if (std::isnan(msg->ranges[index]) or msg->ranges[index] > msg->range_max)
    {
      continue;
    }
    double delta_theta = index * msg->angle_increment + msg->angle_min;
    double xk = x + msg->ranges[index] * std::cos(theta + delta_theta);
    double yk = y + msg->ranges[index] * std::sin(theta + delta_theta);
    if (std::sqrt( std::pow(xk-last_point.first, 2) + std::pow(yk-last_point.second, 2)) > _map.get_resolution())
    {
      if (!(xk > _map.get_x_max() + 2*_map.get_resolution() or
          xk < _map.get_x_min() - 2*_map.get_resolution() or
          yk > _map.get_y_max() + 2*_map.get_resolution() or
          yk < _map.get_y_min() - 2*_map.get_resolution()))
      {
        double z = _map.get_distance_to_closest_obstacle(std::make_pair(xk, yk)) / _z_var;
        double new_prob = (std::erf(z + 2 * resolution) - std::erf(z - 2 * resolution)) / 2 + _z_rand / msg->range_max;
        probability += new_prob;
        count++;
        last_point = std::make_pair(xk, yk);
        /*
        if (probability < DBL_EPSILON)
        {
          probability = 0;
          index = msg->ranges.size();
        }
        */
      }
    }

  }

  return probability/(double) count;
}
