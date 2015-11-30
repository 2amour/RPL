#include "object_recognition_behaviour.h"

ObjectRecognitionBehaviour::ObjectRecognitionBehaviour() {
  is_requested = false;
  _requested_number = 0;
}

ObjectRecognitionBehaviour::ObjectRecognitionBehaviour(ObjectRecognitionPipeline recognition_pipeline)
{
  ROS_INFO("Pipeline set!");
  _recognition_pipeline = recognition_pipeline;
  is_requested = false;
  _requested_number = 0;
}

void ObjectRecognitionBehaviour::image_callback(const PointCloud::ConstPtr& msg)
{
  if (is_requested){
    ros::Time timestamp = ros::Time::now();
    ROS_INFO("I heard a new msg!");
    _listener.msg_callback(msg);
    _recognition_pipeline.pre_process_image(_listener.get_cloud());
    std::vector<PointCloud> clusters = _recognition_pipeline.get_clusters();

    for (std::vector<PointCloud>::iterator it = clusters.begin(); it != clusters.end(); ++it){
      Eigen::Vector4f min, max, mean, difference;
      pcl::getMinMax3D(*it, min, max);
      mean = 0.5 * (min + max);
      difference = max - min;

      _recognition_pipeline.recognize_image(*it);
      publish(mean, difference, _requested_number++, timestamp);
    }
    ROS_INFO("Finished processing point cloud");
    is_requested = false;
  }
}

void ObjectRecognitionBehaviour::set_marker_publisher(ros::Publisher marker_publisher)
{
  _marker_publisher = marker_publisher;
}

void ObjectRecognitionBehaviour::set_pipeline(ObjectRecognitionPipeline recognition_pipeline)
{
  _recognition_pipeline = recognition_pipeline;
  ROS_INFO("Pipeline set!");
}

void ObjectRecognitionBehaviour::publish(Eigen::Vector4f position, Eigen::Vector4f scale, int marker_id, ros::Time timestamp)
{
  std::vector<Category> categories = _recognition_pipeline.get_categories();
  for (std::vector<Category>::iterator it = categories.begin(); it != categories.end(); ++it)
  {
    if (it->is_detected())
    {
      MarkerMessage marker;
      marker.set_position(static_cast<double>(position[0]), static_cast<double>(position[1]),
                          static_cast<double>(position[2]));
      marker.set_scale(static_cast<double>(scale[0]), static_cast<double>(scale[1]), static_cast<double>(scale[2]));
      marker.set_id(marker_id);
      marker.set_color(it->get_color());
      marker.set_frame_id(_frame);
      marker.set_timestamp(timestamp);
      _markers.push_back(marker);
      _marker_publisher.publish(marker.get_marker());
    }
  }
}

void ObjectRecognitionBehaviour::reset_publisher(void)
{
  MarkerMessage marker;
  marker.set_frame_id(_frame);
  marker.set_action(DELETE_ALL);
  _marker_publisher.publish(marker.get_marker());
}

void ObjectRecognitionBehaviour::set_image_frame(const std::string frame)
{
  _frame = frame;
}

void ObjectRecognitionBehaviour::request_callback(const std_msgs::EmptyPtr & msg){
  if (!is_requested){
    ROS_INFO("Request processing");
    is_requested = true;
    ++_requested_number;
  }
}
