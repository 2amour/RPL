#include <ros/ros.h>
#include <string>
#include <iostream>
#include <memory>

#include <map>

#include "object_recognition/behaviour/object_recognition_behaviour.h"
#include <visualization_msgs/Marker.h>


#include "object_recognition/filtering_strategies/filter.h"
#include "object_recognition/segmentation_strategies/segmentation.h"
#include "object_recognition/correspondence_strategies/correspondence.h"
#include "object_recognition/descriptors/spin_image.h"
#include "object_recognition/parsers/ros_categories_parser.h"
#include "object_recognition/parsers/ros_parameters_parser.h"
#include "object_recognition/parsers/ros_topics_parser.h"


#include "object_recognition/util/category.h"

static const std::string IMAGE_LISTENER = "image_topic";
static const std::string REQUEST_LISTENER = "request_topic";
static const std::string IMAGE_RECIEVED_PUBLISHER = "image_recieved_topic";

static const std::string MARKER_PUBLISHER = "marker_topic";
static const std::string NODE_NAME = "object_recognition"; ///< @brief global node name


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  /****************************** Parse categories *******************************************/
  RosCategoriesParser categories_parser(nh);
  std::vector<Category> models = categories_parser.get_categories();

  /**************************** Parse parameters *****************************************/

  RosParameterParser rosparam_parser(nh);
  std::vector<FilterPtr> filters = rosparam_parser.get_filters();
  SegmentationPtr segmentator = rosparam_parser.get_segmentator();
  DescriptorPtr spin_image = rosparam_parser.get_descriptor();
  CorrespondencePtr correspondence = rosparam_parser.get_correspondence();
  std::string frame = rosparam_parser.get_image_frame();

  /**************************** Parse topics *****************************************/
  RosTopicParser image_topic(nh, IMAGE_LISTENER);
  RosTopicParser request_topic(nh, REQUEST_LISTENER);
  RosTopicParser marker_topic(nh, MARKER_PUBLISHER);
  RosTopicParser image_recieved_topic(nh, IMAGE_RECIEVED_PUBLISHER);
  /**********************************************************************************/

  /* ROS BEHAVIOUR */

  ObjectRecognitionPipeline pipeline;
  pipeline.set_filters(filters);
  pipeline.set_segmentator(segmentator);
  pipeline.set_descriptor(spin_image);
  pipeline.set_correspondence(correspondence);
  pipeline.set_models(models);


  ObjectRecognitionBehaviour behaviour(pipeline);
  //behaviour.set_pipeline(pipeline);
  behaviour.set_image_frame(frame);

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(marker_topic.get_topic_name(), marker_topic.get_queue_size());
  behaviour.set_marker_publisher(marker_pub);
  ros::Publisher image_recieved_pub = nh.advertise<std_msgs::Empty>(image_recieved_topic.get_topic_name(), image_recieved_topic.get_queue_size());
  behaviour.set_image_recieved_publisher(image_recieved_pub);

  ros::Subscriber im_sub = nh.subscribe(image_topic.get_topic_name(), image_topic.get_queue_size(), &ObjectRecognitionBehaviour::image_callback,
                                     &behaviour);
  ros::Subscriber re_sub = nh.subscribe(request_topic.get_topic_name(), request_topic.get_queue_size(), &ObjectRecognitionBehaviour::request_callback,
                                       &behaviour);

  ros::spin();

  return 0;
}
