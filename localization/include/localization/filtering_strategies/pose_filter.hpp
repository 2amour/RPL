/**
 * @file pose_filter.hpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs.
 */


void PoseFilter::set_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  _sensor_update_strategy->set_map(msg);
  _map.header = msg->header;
  _map.info = msg->info;
  _map.data = msg->data;
  _is_map_set = true;
}
