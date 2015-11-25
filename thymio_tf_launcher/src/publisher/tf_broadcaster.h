/**
 * @file tf_broadcaster.h
 *
 * This file provides a class definition that broadcasts tfs
 */

#include <vector>
#include <tf/transform_broadcaster.h>


#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

class TfBroadcaster{
private:
  tf::TransformBroadcaster _br; ///< broadcaster object
  std::vector<tf::StampedTransform> _tfs; ///< vector with transforms
public:
  /**
   * Class Constructor
   * @param tfs vector of transforms to publish
   */
  TfBroadcaster();
  void set_tfs(std::vector<tf::StampedTransform> tfs);

  void timerCallback(const ros::TimerEvent& event); ///< timer callback.
};



#endif /* TF_BROADCASTER_H_ */
