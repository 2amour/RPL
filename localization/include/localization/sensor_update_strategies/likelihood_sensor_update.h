/**
 * @file likelihood_sensor_update.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include "sensor_update.h"

#ifndef LIKELIHOOD_SENSOR_UPDATE_H_
#define LIKELIHOOD_SENSOR_UPDATE_H_

/**
 * @brief Gaussian likelihood model implementation of sensor update algorithm.
 *
 */
class LikelihoodSensorUpdate : public SensorUpdate
{
public:
  LikelihoodSensorUpdate(); ///< Default Constructor

  /**
   * Class constructor
   * @param z_rand measurement bias.
   * @param z_var measurement variance .
   */
  LikelihoodSensorUpdate(double z_rand, double z_var);

  /**
   * Get a posteriori probability of being in a pose given the msg.
   * @param a_priori a priori probability of being in a pose.
   * @param a_priori_map a priori pointer to believed map.
   * @param pose believed pose.
   * @param msg recieved scan msg.
   * @return a posteriori probability
   */
  double a_posteriori(double a_priori, OccupancyMapPtr a_priori_map, geometry_msgs::PoseWithCovarianceStamped pose, const sensor_msgs::LaserScan::ConstPtr& msg);
private:
  double _z_rand; ///< random bias.
  double _z_var; ///< measurement variance.
};

#endif /* LIKELIHOOD_SENSOR_UPDATE_H_ */
