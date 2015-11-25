/*
 * tf_broadcaster.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: Sebastian Curi
 */

#include "tf_broadcaster.h"

TfBroadcaster::TfBroadcaster(){}

void TfBroadcaster::set_tfs(std::vector<tf::StampedTransform> tfs)
{
  _tfs = tfs;
}

void TfBroadcaster::timerCallback(const ros::TimerEvent& event)
{
  for (std::vector<tf::StampedTransform>::iterator it = _tfs.begin(); it != _tfs.end(); ++it)
  {
    it->stamp_ = ros::Time::now();
    _br.sendTransform(*it);
  }
}
