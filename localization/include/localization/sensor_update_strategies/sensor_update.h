/**
 * @file sensor_update.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */
#include <cmath>       /* erf */
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "localization/util/occupancy_map.h"

#ifndef SENSOR_UPDATE_H_
#define SENSOR_UPDATE_H_


/**
 * @brief Abstract class for implementing the sensor update strategy pattern
 *
 */
class SensorUpdate
{
public:
  virtual ~SensorUpdate(){}; ///< Virtual Deconstructor

  /**
   * Get a posteriori probability of being in a pose given the msg.
   * @param a_priori a priori probability of being in a pose.
   * @param a_priori_map a priori pointer to believed map.
   * @param pose believed pose.
   * @param msg recieved scan msg.
   * @return a posteriori probability
   */
  virtual double a_posteriori(double a_priori, OccupancyMapPtr a_priori_map, geometry_msgs::PoseWithCovarianceStamped pose, const sensor_msgs::LaserScan::ConstPtr& msg) = 0;

  /**
   * Set global map to the update algorithm.
   * @param map Occupancy grid msg pointer.
   */
  void set_map(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /**
   * Get algorithm name.
   * @return
   */
  std::string get_name();

  /**
   * Get reference frame of update step.
   * @return
   */
  std::string get_reference_frame();
protected:
  std::string _name; ///< algorithm name.
  OccupancyMap _map; ///< global map.
};

/**
 * @brief Shared pointer to sensor_update class
 */
typedef boost::shared_ptr<SensorUpdate> SensorUpdatePtr;


#endif /* SENSOR_UPDATE_H_ */
