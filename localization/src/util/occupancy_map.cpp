/**
 * @file ocuppancy_map.cpp
 *
 *  @date Dec 6, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include <localization/util/occupancy_map.h>

static const int THRESHOLD = 50;

OccupancyMap::OccupancyMap()
{
  _map = nav_msgs::OccupancyGrid();
}

OccupancyMap::OccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  _map.data = map->data;
  _map.info = map->info;
  _map.header = map->header;
  generate_obstacle_list();
}

nav_msgs::OccupancyGrid OccupancyMap::get_map()
{
  return _map;
}

double OccupancyMap::get_resolution()
{
  return _map.info.resolution;
}

void OccupancyMap::set_map(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  _map.data = map->data;
  _map.info = map->info;
  _map.header = map->header;
  generate_obstacle_list();
}

void OccupancyMap::create_map(int width, int height, double resolution)
{
  _map = nav_msgs::OccupancyGrid();
  _map.info.width = width;
  _map.info.height = height;
  _map.info.resolution = resolution;
  if (_map.data.size() < width * height)
  {
    _map.data.assign(width * height, 0.0);
  }
  else
  {
    reset();
  }
}

void OccupancyMap::reset()
{
  for (int i = 0; i < _map.data.size(); ++i)
  {
    _map.data[i] = 0.0;
  }
}

void OccupancyMap::set_value(int index, double value)
{
  _map.data[index] = value;
}

void OccupancyMap::set_value(std::pair<double, double> point, double value)
{
  if (point.first > _map.info.origin.position.x and point.first < _map.info.resolution * _map.info.width
      and point.second > _map.info.origin.position.y and point.second < _map.info.resolution * _map.info.height)
  {
    int index = get_index_from_point(point);
    if (index > 0 and index < _map.data.size())
    {
      _map.data[index] = value;
    }
  }

}

std::string OccupancyMap::get_reference_frame()
{
  return _map.header.frame_id;
}

std::vector<std::pair<double, double> > OccupancyMap::get_obstacle_list()
{
  return _obstacle_list;
}

double OccupancyMap::get_distance_to_closest_obstacle(std::pair<double, double> point)
{
  double distance = _map.info.resolution
      * std::sqrt(_map.info.width * _map.info.width + _map.info.height * _map.info.height);
  double new_distance;
  for (auto p_point = _obstacle_list.begin(); p_point < _obstacle_list.end(); ++p_point)
  {
    new_distance = std::sqrt(std::pow(p_point->second - point.second, 2) + std::pow(p_point->first - point.first, 2));
    if (new_distance < distance)
    {
      distance = new_distance;
    }
  }
  return distance;
}

int OccupancyMap::get_index_from_point(std::pair<double, double> point)
{
  int ix = std::round((point.first - _map.info.origin.position.x) / (_map.info.resolution));
  int iy = std::round((point.second - _map.info.origin.position.y) / (_map.info.resolution));
  int index = ix + iy * _map.info.width;
  if (index > _map.data.size() or index < 0)
  {
    index = -1;
  }
  return index;
}

std::pair<double, double> OccupancyMap::get_coordinates_from_index(int index)
{
  double x = _map.info.origin.position.x + (index % _map.info.width) * _map.info.resolution;
  double y = _map.info.origin.position.y + (index / _map.info.width) * _map.info.resolution;
  return std::make_pair(x, y);
}

void OccupancyMap::generate_obstacle_list()
{
  _obstacle_list.clear();
  for (int i = 0; i < _map.data.size(); ++i)
  {
    if (_map.data[i] > THRESHOLD)
    {
      _obstacle_list.push_back(get_coordinates_from_index(i));
    }
  }
}

double OccupancyMap::correlation(OccupancyMap other)
{
  int size = _map.data.size();
  nav_msgs::OccupancyGrid other_map = other.get_map();

  double average_map = 0.0;
  double sum_this, sum_other, squared_this, squared_other, product  = 0.0;
  for (int i = 0; i < _map.data.size(); ++i)
  {
    sum_this += _map.data[i];
    squared_this += std::pow(_map.data[i], 2);
    sum_other += other_map.data[i];
    squared_other += std::pow(other_map.data[i], 2);
    product += _map.data[i] * other_map.data[i];
  }

  double correlation = (size*product - sum_this*sum_other)/(std::sqrt(size*squared_this - std::pow(sum_this, 2)) *
      std::sqrt(size*squared_other - std::pow(sum_other, 2)));

  if (correlation < 0 or std::isnan(correlation))
  {
    return 0;
  }
  return correlation;

}

double OccupancyMap::get_x_min()
{
  return _map.info.origin.position.x;
}
double OccupancyMap::get_y_min()
{
  return _map.info.origin.position.y;
}
double OccupancyMap::get_x_max()
{
  return _map.info.origin.position.x + _map.info.width * _map.info.resolution;
}
double OccupancyMap::get_y_max()
{
  return _map.info.origin.position.x + _map.info.height * _map.info.resolution;
}
double OccupancyMap::get_height()
{
  return _map.info.height;
}
double OccupancyMap::get_width()
{
  return _map.info.width;
}
