/** @file object_recognition_behaviour.h
 *  @brief Class that implements the behaviour of the obstacle recognition algorithm..
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */


#include <ros/ros.h>
#include <vector>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "../filtering_strategies/filter.h"
#include "../segmentation_strategies/segmentation.h"
#include "../correspondence_strategies/correspondence.h"
#include "../descriptors/spin_image.h"
#include "../types/points.h"
#include "../listeners/point_cloud_listener.h"
#include "../msg/visualization_marker.h"
#include "../util/category.h"

static const int DELETE_ALL = 3; ///< @brief Key to delete all markers. It should be implemented in visualization_msgs::Marker::DELETEALL.

/**
 * @brief Class that implements the object recognition algorithm.
 */
class ObjectRecognitionBehaviour {
private:
  ros::Publisher _marker_publisher; ///< Publisher of ROS Messages
  PointCloudListener _listener; ///< Listener of ROS Messages.
  std::vector<FilterPtr> _filters; ///< Array of pointers to abstract filters strategy.
  SegmentationPtr _segmentator; ///<Pointer to abstract segmentation strategy.
  DescriptorPtr _spin_image; ///<Pointer to SpinImage generator.
  std::vector<Category> _categories; ///< Array of categories
  CorrespondencePtr _correspondence;  ///< Pointer to abstract correspondence strategy.
  std::vector<MarkerMessage> _markers; ///< Vector of Marker messages.
  void publish(Eigen::Vector4f position, Eigen::Vector4f scale, int cluster_number); ///< Publisher wrapper.
  void reset_publisher(void); ///< Reset publisher objects
  std::string _frame; ///< Image frame
public:
  ObjectRecognitionBehaviour() {}; ///< Default constructor.
  /**
   * Class constructor.
   * @param filters Array of pointers to implemented filters strategy.
   * @param segmentator Pointer to abstract implemented strategy.
   * @param spin_image Pointer to SpinImage generator.
   * @param correspondence Pointer to implemented correspondence strategy.
   */
  ObjectRecognitionBehaviour(std::vector<FilterPtr> filters, SegmentationPtr segmentator, DescriptorPtr spin_image, CorrespondencePtr correspondence);

  /**
   * Set sequence of filters to pre-process the image
   * @param filters vector of filters
   */
  void set_filters(std::vector<FilterPtr> filters);

  /**
   * Set segmentation strategy to cluster the filtered cloud
   * @param segmentator
   */
  void set_segmentator(SegmentationPtr segmentator);

  /**
   * Set the descriptor for the image recognition
   * @param spin_image
   */
  void set_descriptor(DescriptorPtr spin_image);

  /**
   * Set the correspondence strategy to match descriptors
   * @param correspondence
   */
  void set_correspondence(CorrespondencePtr correspondence);

  /**
   * Set an advertised marker publisher.
   * @param marker_publisher advertised publisher to ros node_handler.
   */
  void set_marker_publisher(ros::Publisher marker_publisher);

  /**
   * Set vector of known models
   * @param categories
   */
  void set_models(std::vector<Category> categories);

  /**
   * Callback when a new image is recieved.
   * @param msg
   */
  void image_callback(const PointCloud::ConstPtr& msg);

  /**
   * Pre-process image method. Here the filters and segmentation algorithms are implemented.
   * @param point_cloud point_cloud to pre-process.
   */
  void pre_process_image(const PointCloud & point_cloud);

  /**
   * Recognize algorithm. Here the spin-image of the point_cloud is computed and the correspondence algorithm is performed.
   * @param point_cloud point cloud to be recognized
   * @param cluster_id id of marker tu publish if recognized!
   */
  void recognize_image(const PointCloud & point_cloud, int cluster_id);

  /**
   * Set the frame where the object recognition is done.
   * @param frame image frame.
   */
  void set_image_frame(const std::string frame);
};
