/** @file category.h
 *  @brief Class for encapsulating training models.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "../descriptors/spin_image.h"
#include "../msg/visualization_marker.h"

#ifndef _CATEGORY_H_
#define _CATEGORY_H_

/**
 * @brief Encapsulation of image models data.
 */
class Category{
private:
  std::string _name; ///< category name.
  //MarkerMessage _msg; ///< msg to publish a detection.
  struct Color _color; ///< color
  std::vector<DescriptorCloud> _learning_set; ///< vector of knwon images of this category.
  bool _is_detected; ///< boolean to check if a detection was performed.
public:
  Category(); ///< Default class constructor.
   /**
    * Class constructor
    * @param name name of category
    */
  Category(const std::string & name);

  /**
   * Add a new instance of the model.
   * @param _learning_set a new point cloud of known models.
   */
  void add_learning_set(const DescriptorCloud & _learning_set);

  /**
   * Add a new instance of the model.
   * @param path a path to a .pcd file with a point cloud of known models.
   */
  void add_learning_set(const std::string & path);

  /**
   * Set if detection occurred or not.
   * @param is_detected
   */
  void set_detected(bool is_detected);


  /**
   * Set marker color by RGB values.
   * @param r
   * @param g
   * @param b
   */
  void set_color(double r, double g, double b);

  /**
   * Set color of this category.
   * @param color structure of rgb color
   */
  void set_color(struct Color color);

  /**
   * Get Color of this category.
   * @return struct Color color.
   */
  struct Color get_color(void);

  /**
   * Set category name.
   * @param name category name
   */
  void set_name(const std::string & name);

  /**
   * Check if object is detected
   * @return
   */
  bool is_detected(void);

  /**
   * Get category name.
   * @return category name
   */
  const std::string & get_name(void);

  /**
   * Get vector of knwon point clouds.
   * @return vector of knwon point clouds.
   */
  std::vector<DescriptorCloud> get_learning_set(void);
};




#endif /* _CATEGORY_H_ */
