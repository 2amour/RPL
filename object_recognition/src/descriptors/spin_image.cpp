/*
 * spin_image.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "../descriptors/spin_image.h"
SpinImage::SpinImage()
{
  spin_image_params params = spin_image_params();
  _image_width = params.image_width;
  _support_angle_cos = params.support_angle_cos;
  _min_pts_neighb = params.min_pts_neighb;
  _radius = params.radius;
  _max_points = params.max_points;
}

SpinImage::SpinImage(struct spin_image_params params)
{
  _image_width = params.image_width;
  _support_angle_cos = params.support_angle_cos;
  _min_pts_neighb = params.min_pts_neighb;
  _radius = params.radius;
  _max_points = params.max_points;
}

SpinImage::SpinImage(unsigned int image_width, double support_angle_cos, unsigned int min_pts_neighb, double radius, unsigned int max_points) :
    _image_width(image_width), _support_angle_cos(support_angle_cos), _min_pts_neighb(min_pts_neighb), _radius(radius), _max_points(max_points)
{
}

void SpinImage::set_image_width(int image_width)
{
  _image_width = image_width;
}
void SpinImage::set_support_angle_cos(double support_angle_cos)
{
  _support_angle_cos = support_angle_cos;
}
void SpinImage::set_min_points_neighbour(int min_pts_neighb)
{
  _min_pts_neighb = min_pts_neighb;
}
void SpinImage::set_radius(double radius)
{
  _radius = radius;
}

DescriptorCloud SpinImage::compute_spin_image(const PointCloud::Ptr & cloud)
{
  PointCloud::Ptr pc = cloud;
  _filter = VoxelGridFilter();
  while (pc->size()>_max_points){
    _filter.set_size(_filter.get_size()*1.5);
    _filter.filter(pc);
    pc.reset(new PointCloud(_filter.get_filtered_cloud()));
  }
  // Compute the normals
  pcl::NormalEstimation<PointType, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud(pc);
  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  normal_estimation.setSearchMethod(kdtree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimation.setRadiusSearch(_radius);
  normal_estimation.compute(*normals);

  // Setup spin image computation
  pcl::SpinImageEstimation<PointType, pcl::Normal, DescriptorType> spin_image_descriptor(_image_width,
                                                                                             _support_angle_cos,
                                                                                             _min_pts_neighb);
  spin_image_descriptor.setInputCloud(pc);
  spin_image_descriptor.setInputNormals(normals);

  // Use the same KdTree from the normal estimation
  spin_image_descriptor.setSearchMethod(kdtree);
  DescriptorCloud spin_images;

  spin_image_descriptor.setRadiusSearch(_radius);

  // Actually compute the spin images
  spin_image_descriptor.compute(spin_images);

  return spin_images;
}

