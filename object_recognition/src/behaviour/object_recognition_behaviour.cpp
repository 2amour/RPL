#include "object_recognition_behaviour.h"

ObjectRecognitionBehaviour::ObjectRecognitionBehaviour(std::vector<FilterPtr> filters, SegmentationPtr segmentator,
                                                       DescriptorPtr spin_image, CorrespondencePtr correspondence) :
    _filters(filters), _segmentator(segmentator), _spin_image(spin_image), _correspondence(correspondence)
{
}

void ObjectRecognitionBehaviour::set_filters(std::vector<FilterPtr> filters){
  _filters = filters;
}

void ObjectRecognitionBehaviour::set_segmentator(SegmentationPtr segmentator){
  _segmentator = segmentator;
}

void ObjectRecognitionBehaviour::set_descriptor(DescriptorPtr spin_image){
  _spin_image = spin_image;
}

void ObjectRecognitionBehaviour::set_correspondence(CorrespondencePtr correspondence){
  _correspondence = correspondence;
}


void ObjectRecognitionBehaviour::image_callback(const PointCloud::ConstPtr& msg)
{
  ROS_INFO("I heard a new msg!");
  _listener.msg_callback(msg);

  pre_process_image(_listener.get_cloud());
  if ((_segmentator->get_cluster_indices()).size() != _markers.size())
  {
    reset_publisher();
    ROS_WARN("Errasing markers");
  }
  _markers.clear();
  for (size_t i = 0; i < (_segmentator->get_cluster_indices()).size(); ++i)
  {
    recognize_image(_segmentator->get_cluster_cloud(i), i);
  }
  ROS_INFO("Finished processing point cloud");
}

void ObjectRecognitionBehaviour::pre_process_image(const PointCloud & point_cloud)
{
  PointCloud pc(point_cloud);
  for (std::vector<FilterPtr>::iterator it = _filters.begin(); it != _filters.end(); ++it)
  {
    ((*it).get())->filter(pc.makeShared());
    pc = ((*it).get())->get_filtered_cloud();
  }
  _segmentator->extract_clusters(pc);
}

void ObjectRecognitionBehaviour::recognize_image(const PointCloud & point_cloud, int cluster_id)
{
  Eigen::Vector4f min, max, mean, difference;
  pcl::getMinMax3D(point_cloud, min, max);
  mean = 0.5 * (min + max);
  difference = max - min;
  for (std::vector<Category>::iterator it = _categories.begin(); it != _categories.end(); ++it)
  {
    it->set_detected(false);
  }
  bool has_matched = false;
  try
  {
    DescriptorCloud output = _spin_image->compute_spin_image(point_cloud.makeShared());
    for (std::vector<Category>::iterator it = _categories.begin(); it != _categories.end() - 1; ++it)
    {
      bool detected = false;
      std::vector<DescriptorCloud> learning_set = it->get_learning_set();
      for (std::vector<DescriptorCloud>::iterator ls_it = learning_set.begin(); ls_it != learning_set.end(); ++ls_it)
      {
        _correspondence->match(*ls_it, output);
        detected |= _correspondence->has_matched();
        ROS_INFO("Cluster %d matched with category %s with %f percentage", cluster_id, (it->get_name()).c_str(),
                 _correspondence->get_match_percentage());
        it->set_detected(detected);
        has_matched |= detected;
        if (detected)
        {
          break;
        }
      }
      if (has_matched)
      {
        break;
      }
    }

  }
  catch (const pcl::PCLException& e)
  {
    ROS_WARN("%s", e.what());
  }
  //set_unknown:
  _categories.back().set_detected(!has_matched);
  publish(mean, difference, cluster_id);
}

void ObjectRecognitionBehaviour::set_marker_publisher(ros::Publisher marker_publisher)
{
  _marker_publisher = marker_publisher;
}

void ObjectRecognitionBehaviour::set_models(std::vector<Category> categories)
{
  _categories = categories;
  ROS_INFO("Categories set!");
}

void ObjectRecognitionBehaviour::publish(Eigen::Vector4f position, Eigen::Vector4f scale, int cluster_number)
{
  for (std::vector<Category>::iterator it = _categories.begin(); it != _categories.end(); ++it)
  {
    if (it->is_detected())
    {
      MarkerMessage marker;
      marker.set_position(static_cast<double>(position[0]), static_cast<double>(position[1]),
                          static_cast<double>(position[2]));
      marker.set_scale(static_cast<double>(scale[0]), static_cast<double>(scale[1]), static_cast<double>(scale[2]));
      marker.set_id(cluster_number);
      marker.set_color(it->get_color());
      marker.set_frame_id(_frame);
      _markers.push_back(marker);
      _marker_publisher.publish(marker.get_marker());
    }
  }
}

void ObjectRecognitionBehaviour::reset_publisher(void)
{
  /*
   for (std::vector<MarkerMessage>::iterator it = _markers.begin(); it != _markers.end(); ++it){
   it->set_action(visualization_msgs::Marker::DELETE);
   _marker_publisher.publish(it->get_marker());
   }*/
  MarkerMessage marker;
  marker.set_frame_id(_frame);
  marker.set_action(DELETE_ALL);
  _marker_publisher.publish(marker.get_marker());
}

void ObjectRecognitionBehaviour::set_image_frame(const std::string frame)
{
  _frame = frame;
}
