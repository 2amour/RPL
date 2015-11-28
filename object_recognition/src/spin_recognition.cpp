#include <ros/ros.h>
#include <string>
#include <iostream>
#include <memory>

#include <map>

#include "behaviour/object_recognition_behaviour.h"
#include <visualization_msgs/Marker.h>
#include "ros/topics.h"


#include "filtering_strategies/filter.h"
#include "segmentation_strategies/segmentation.h"
#include "correspondence_strategies/correspondence.h"
#include "descriptors/spin_image.h"
#include "parsers/model_parser.h"
#include "parsers/ros_parameters_parser.h"

#include "util/category.h"


int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage ./spin_training file_with_classes" << std::endl;
    return -1;
  }
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  /****************************** PARSING *******************************************/
  std::string classes_file = argv[1];
  std::vector<Category> models = get_models(classes_file);

  RosParameterParser rosparam_parser(nh);
  std::vector<FilterPtr> filters = rosparam_parser.get_filters();
  SegmentationPtr segmentator = rosparam_parser.get_segmentator();
  DescriptorPtr spin_image = rosparam_parser.get_descriptor();
  CorrespondencePtr correspondence = rosparam_parser.get_correspondence();
  std::string frame = rosparam_parser.get_image_frame();

  /**********************************************************************************/

  /* ROS BEHAVIOUR */

  ObjectRecognitionBehaviour behaviour;
  behaviour.set_filters(filters);
  behaviour.set_segmentator(segmentator);
  behaviour.set_descriptor(spin_image);
  behaviour.set_correspondence(correspondence);
  behaviour.set_models(models);
  behaviour.set_image_frame(frame);

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( MARKER_TOPIC, MARKER_QUEUE_SIZE);
  behaviour.set_marker_publisher(marker_pub);


  ros::Subscriber im_sub = nh.subscribe(IMAGE_TOPIC, IMAGE_QUEUE_SIZE, &ObjectRecognitionBehaviour::image_callback,
                                     &behaviour);
  ros::Subscriber re_sub = nh.subscribe(REQUEST_TOPIC, REQUEST_QUEUE_SIZE, &ObjectRecognitionBehaviour::request_callback,
                                       &behaviour);

  ros::spin();

  return 0;
}
