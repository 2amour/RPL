/**
 * @file pose_filter.h
 *
 *  This is an abstract class for implementing a localization filter algorithm
 *  using a strategy pattern.
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>

#include "localization/sensor_update_strategies/sensor_update.h"
#include "localization/motion_update_strategies/motion_update.h"
#include "localization/sampling_strategies/sampling.h"

#ifndef POSE_FILTER_H_
#define POSE_FILTER_H_

/**
 * @brief Abstract class for implementing the pose filter strategy pattern
 *
 */
class PoseFilter
{
public:
  virtual ~PoseFilter(){}; ///< Virtual Destructor

  /**
   * Execute motion update step.
   * @param msg odometry msg pointer.
   */
  virtual void motion_update(const nav_msgs::Odometry::ConstPtr& msg) = 0;

  /**
   * Execute sensor correcting step.
   * @param msg laser msg pointer.
   */
  virtual void sensor_update(const sensor_msgs::LaserScan::ConstPtr& msg) = 0;

  /**
   * Get all the estimated poses.
   * @return
   */
  virtual std::vector<geometry_msgs::PoseWithCovarianceStamped> get_poses() = 0;

  /**
   * Get the best estimation of the pose.
   * @return
   */
  virtual geometry_msgs::PoseWithCovarianceStamped get_best_pose() = 0;

  /**
   * Set the motion model implementation.
   * @param ptr motion update pointer.
   */
  void set_motion_update_strategy(MotionUpdatePtr ptr);

  /**
   * Set the sensor model implementation.
   * @param ptr sensor update pointer.
   */
  void set_sensor_update_strategy(SensorUpdatePtr ptr);

  /**
   * Set the sampling strategy implementation.
   * @param ptr sampling strategy pointer.
   */
  void set_sampling_strategy(SamplingPtr ptr);

  /**
   * Set map to the algorithm
   * @param msg occupancy grid msg pointer.
   */
  virtual void set_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
   * Return algorithm name.
   * @return
   */
  std::string get_name();

  /**
   * Return algorithm reference frame.
   * @return
   */
  std::string get_frame();

  /**
   * Check if filter is confident.
   * @return
   */
  virtual bool is_good() = 0;
protected:
  std::string _name; ///< algorithm name.
  MotionUpdatePtr _motion_update_strategy; ///< motion update model.
  SensorUpdatePtr _sensor_update_strategy; ///< sensor update model.
  SamplingPtr _sampling_strategy; ///< sampling strategy model.
  nav_msgs::OccupancyGrid _map; ///< global map model.
  bool _is_map_set; ///< boolean to check if map is set.
};


/**
 * @brief Shared pointer to Pose filter class
 */
typedef boost::shared_ptr<PoseFilter> PoseFilterPtr;

#include "pose_filter.hpp"
#endif /* POSE_FILTER_H_ */
