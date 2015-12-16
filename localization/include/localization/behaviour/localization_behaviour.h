/**
 * @file localization_behaviour.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */
#include <ros/ros.h>
#include "localization/filtering_strategies/particle_filter.h"
#include "localization/util/particle.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>


#ifndef LOCALIZATION_BEHAVIOUR_H_
#define LOCALIZATION_BEHAVIOUR_H_

/**
 * @brief Class that sequences the pose filter algorithm from ros msgs and publishes the result.
 *
 */
class LocalizationBehaviour
{
public:
  LocalizationBehaviour(); ///< Default Constructor

  /**
   * Class constructor from pose filter
   * @param filter filter implementation
   */
  LocalizationBehaviour(PoseFilterPtr filter);

  /**
   * Set the filter implementation
   * @param filter pose filter implementation.
   */
  void set_filter(PoseFilterPtr filter);

  /**
   * Callback method when the map is recieved or updated.
   * @param msg map msg pointer.
   */
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
   * Callback method when the odometry msg is recieved. The motion update is called from within.
   * @param msg odometry msg pointer.
   */
  void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * Callback method when the laser msg is recieved. The sensor update is called from within.
   * @param msg sensor msg pointer.
   */
  void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * Request localization algorihtm
   * @param msg
   */
  void request_callback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Publish an array of poses.
   * @param arrays Array of poses to publish.
   * @param header Header with time and frame id of the arrays.
   */
  void publish_array(std::vector<geometry_msgs::PoseWithCovarianceStamped> arrays, std_msgs::Header header);

  /**
   * Publish a tf between the map and the estimated pose.
   * @param pose estimated pose.
   * @param header Header with time and frame id of the pose.
   */
  void publish_estimated_tf(geometry_msgs::PoseWithCovarianceStamped pose, std_msgs::Header header);

  /**
   * Publish odometry
   * @param odom estimated odometry
   * @param header header where to publish
   */
  void publish_odometry(geometry_msgs::PoseWithCovarianceStamped pose, std_msgs::Header header);

  /** Status publisher
   *
   * @param is_localized
   */
  void publish_status(bool is_localized);

  /**
   * Setter of the array publisher.
   * @param pose_array_pub
   */
  void set_array_publisher(ros::Publisher pose_array_pub);

  /**
   * Setter of odometry publisher
   * @param odometry_pub
   */
  void set_odometry_publisher(ros::Publisher odometry_pub);

  /**
   * Setter of status publisher
   * @param status_pub
   */
  void set_status_publisher(ros::Publisher status_pub);



private:
  PoseFilterPtr _filter; ///< Pointer to filter implementation
  ros::Publisher _pose_array_publisher; ///< Pose array publisher
  ros::Publisher _odometry_publisher; ///< odometry publisher
  ros::Publisher _status_publisher; ///< status publisher

  tf::TransformBroadcaster _transform_broadcaster; ///< Tf broadcaster
  bool _is_requested;
  bool _was_localized;
};

#endif /* LOCALIZATION_BEHAVIOUR_H_ */
