/**
 * @file occupancy_map.h
 *
 *  @date Dec 6, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include <cmath>
#include <vector>
#include <utility>
#include <nav_msgs/OccupancyGrid.h>

#ifndef OCUPPANCY_MAP_H_
#define OCUPPANCY_MAP_H_

/**
 * @brief Map class for creating, modifying or finding statistics from graphs.
 *
 */
class OccupancyMap
{
public:
  OccupancyMap(); ///< Default Constructor

  /**
   * Class constructor from ROS occupancy grid msg.
   * @param map occupancy grid msg pointer.
   */
  OccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr& map);

  nav_msgs::OccupancyGrid get_map(); ///< Get copy of local map

  /**
   * Set the map from ROS occupancy grid msg
   * @param map map occupancy grid msg pointer
   */
  void set_map(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /**
   * Create an empty graph with given values
   * @param width number of horizontal cells
   * @param height number of vertical cells
   * @param resolution distance per cell
   */
  void create_map(int width, int height, double resolution);
  /**
   * Set value at a given index of the map.
   * @param index index to access map.
   * @param value value to be set.
   */
  void set_value(int index, double value);
  /**
   * Set value at a given point of the map
   * @param point point(x, y) of the map.
   * @param value value to be set.
   */
  void set_value(std::pair<double, double> point, double value);
  void reset(); ///< Reset to zero all values in the map.

  std::string get_reference_frame(); ///< Get graph reference frame
  double get_resolution(); ///< Get graph resolution
  double get_x_min(); ///< Get graph minimum x coordinate
  double get_y_min(); ///< Get graph minimum y coordinate
  double get_x_max(); ///< Get graph maximum x coordinate
  double get_y_max(); ///< Get graph maximum y coordinate
  double get_height(); ///< Get graph number of vertical cells
  double get_width(); ///< Get graph number of horizontal cells
  std::vector<std::pair<double, double> > get_obstacle_list(); ///< Get list of coordinates (x, y) where there are obstacles
  double get_distance_to_closest_obstacle(std::pair<double, double> point); ///< Get distance between a point and the closest obstacle to it.
  int get_index_from_point(std::pair<double, double> point); ///< Get index where a point (x, y) falls in the graph.
  std::pair<double, double> get_coordinates_from_index(int index); ///< Get coordinates (x, y) of a given index.
  double correlation(OccupancyMap other); ///< Calculate cross correlation between this and another map.


private:
  nav_msgs::OccupancyGrid _map; ///< Local copy of map.
  std::vector<std::pair<double, double> > _obstacle_list; ///< List of coordinates (x, y) where there are obstacles.
  void generate_obstacle_list(); ///< Helper function to generate the obstacle list.
};

/**
 * @brief Shared pointer to ocuppancy_map class
 */
typedef boost::shared_ptr<OccupancyMap> OccupancyMapPtr;

#endif /* OCUPPANCY_MAP_H_ */
