#include "point_cloud_listener.h"

PointCloudListener::PointCloudListener()
{
}

PointCloudListener::~PointCloudListener()
{
}

void PointCloudListener::msg_callback(const PointCloud::ConstPtr & cloud_msg)
{
  _cloud = *cloud_msg;
}

const PointCloud & PointCloudListener::get_cloud()
{
  return _cloud;
}
