/** @file object_recognition_behaviour.h
 *  @brief Class that implements the behaviour of the obstacle recognition algorithm..
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */


#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "object_recognition/types/points.h"
#include "object_recognition/listeners/point_cloud_listener.h"
#include "object_recognition/msg/visualization_marker.h"
#include "object_recognition/pipeline/object_recognition_pipeline.h"

/**
 * @brief Class that implements the object recognition algorithm.
 */
class ObjectRecognitionBehaviour {
private:
  ros::Publisher _marker_publisher; ///< Publisher of ROS Messages
  ros::Publisher _image_recieved_publisher; ///< Publisher of ROS Messages
  PointCloudListener _listener; ///< Listener of ROS Messages.

  ObjectRecognitionPipeline _recognition_pipeline; ///< Object recognition pipeline.

  std::vector<MarkerMessage> _markers; ///< Vector of Marker messages.
  void publish(Eigen::Vector4f position, Eigen::Vector4f scale, int marker_id, ros::Time timestamp); ///< Publisher wrapper.
  void reset_publisher(void); ///< Reset publisher objects
  std::string _frame; ///< Image frame

  bool is_requested; ///< flag to request a new recognition.
  int _requested_number;
public:
  ObjectRecognitionBehaviour(); ///< Default constructor.
  /**
   * Class constructor.
   * @param filters Array of pointers to implemented filters strategy.
   * @param segmentator Pointer to abstract implemented strategy.
   * @param spin_image Pointer to SpinImage generator.
   * @param correspondence Pointer to implemented correspondence strategy.
   */
  ObjectRecognitionBehaviour(ObjectRecognitionPipeline recognition_pipeline);

  /**
   * Set an advertised marker publisher.
   * @param marker_publisher advertised publisher in ros node_handler.
   */
  void set_marker_publisher(ros::Publisher marker_publisher);

  /**
   * Set an advertised image recieved publisher
   * @param image_recieved_publisher advertised publisher in ros node_handler.
   */
  void set_image_recieved_publisher(ros::Publisher image_recieved_publisher);

  /**
   * Set the used recognition pipeline
   * @param recognition_pipeline
   */
  void set_pipeline(ObjectRecognitionPipeline recognition_pipeline);

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

  /** Callback to handle request msgs
   * ROS msg.
   * @param msg
   */
  void request_callback(const std_msgs::EmptyPtr & msg);
};
