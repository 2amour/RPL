/**
 * @file odometry_motion_update.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include <localization/motion_update_strategies/motion_update.h>

#ifndef ODOMETRY_MOTION_UPDATE_H_
#define ODOMETRY_MOTION_UPDATE_H_

/**
 * @brief Odometry model implementation of motion update algorithm.
 *
 */
class OdometryMotionUpdate : public MotionUpdate
{
public:
  OdometryMotionUpdate(); ///< Default Constructor
  /**
   * Predict the new pose.
   * @param pose current pose.
   * @param old_msg last odometry msg.
   * @param new_msg new odometry msg.
   * @return
   */
  geometry_msgs::PoseWithCovarianceStamped predict(geometry_msgs::PoseWithCovarianceStamped pose,
                                                   const nav_msgs::Odometry::ConstPtr& old_msg,
                                                   const nav_msgs::Odometry::ConstPtr& new_msg);
};

#endif /* ODOMETRY_MOTION_UPDATE_H_ */
