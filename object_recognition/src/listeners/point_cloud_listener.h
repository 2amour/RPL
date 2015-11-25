/** @file point_cloud_listener.h
 *  @brief Class for listening to a point cloud from ROS.
 *
 *  This file contains a class for listening from a PointCloud2 msg from ros.
 *  It also defines the type of the Point and the PointCloud.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <sensor_msgs/PointCloud2.h>
#include "../types/points.h"

#ifndef _POINT_CLOUD_LISTENER
#define _POINT_CLOUD_LISTENER

/**
 * @brief Class for listening a ROS msg.
 *
 * It has a msg_callback and stores the read msg in a variable.
 * The read msg can be retrieved with a getter.
 */
class PointCloudListener
{
private:
  PointCloud _cloud; ///< stored point cloud;
public:
  /**
   * Point cloud getter
   * @return the last point cloud read from ROS.
   */
  const PointCloud & get_cloud();

  /**
   * Callback that is called everytime a new msg is recieved.
   * @param msg
   */
  void msg_callback(const PointCloud::ConstPtr& msg);

  /**
   * Default class constructor.
   */
  PointCloudListener();

  /**
   * Class destructor.
   */
  ~PointCloudListener();
};

#endif //_POINT_CLOUD_LISTENER
