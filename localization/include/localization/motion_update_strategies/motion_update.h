/**
 * @file motion_update.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#ifndef MOTION_UPDATE_H_
#define MOTION_UPDATE_H_

/**
 * @brief Abstract class for implementing the motion update strategy pattern
 *
 */
class MotionUpdate
{
public:
  virtual ~MotionUpdate()
  {
  }
  ; ///< Virtual Deconstructor

  /**
   * Predict the new pose.
   * @param pose current pose.
   * @param old_msg last odometry msg.
   * @param new_msg new odometry msg.
   * @return
   */
  virtual geometry_msgs::PoseWithCovarianceStamped predict(geometry_msgs::PoseWithCovarianceStamped pose,
                                                           const nav_msgs::Odometry::ConstPtr& old_msg,
                                                           const nav_msgs::Odometry::ConstPtr& new_msg) = 0;
  /**
   * Get algorithm's name
   * @return
   */
  std::string get_name();

  /**
   * Set prediction variance.
   * @param variance
   */
  void set_variance(std::vector<double> variance);
protected:
  std::string _name; ///< algorithm name.
  std::vector<double> _variance; ///< prediction variance.
  std::random_device rd; ///< random device
  std::mt19937 _gen; ///< random generator.
};

/**
 * @brief Shared pointer to motion_update class
 */
typedef boost::shared_ptr<MotionUpdate> MotionUpdatePtr;

#endif /* MOTION_UPDATE_H_ */
