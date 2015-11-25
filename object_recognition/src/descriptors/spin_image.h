/** @file spin_image.h
 *  @brief Class that implements the Spin Image descriptor.
 *
 *  This file contains a class for implementing the spin-image of a set of points.
 *  It defines a pointer to this class.
 *  It defines the DescriptorType used in the project
 *  It registers the point cloud.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <vector>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#include "../filtering_strategies/voxel_grid_filter.h"
#include "../types/points.h"

#ifndef _SPIN_IMAGE_H_
#define _SPIN_IMAGE_H_

/// @brief Width of spin image.
static const int IMAGE_WIDTH = 8;
/// @brief Number of bins in histogram.
static const int BIN_SIZE = (IMAGE_WIDTH+1)*(2*IMAGE_WIDTH+1);
/// @brief Declaration of Descriptor type.
typedef pcl::Histogram<BIN_SIZE> DescriptorType;
/// @brief Declaration of Descriptor Cloud.
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
/// @brief Registration of DescriptorType to PCL library.
POINT_CLOUD_REGISTER_POINT_STRUCT(DescriptorType, (float[BIN_SIZE], histogram, histogram))

namespace pcl
{
/**
 * @brief Class for transforming DescriptorType to array.
 *
 * This class is used by KdTree when searching for correspondence.
 */
template<>
  class DefaultPointRepresentation<DescriptorType> : public PointRepresentation<DescriptorType>
  {
  public:
    DefaultPointRepresentation()
    {
      nr_dimensions_ = BIN_SIZE;
    }

    /**
     * Transform DescriptorType to an FloatArray representation
     * @param p const reference to descriptor type to get array representation.
     * @param out array float representation of DescriptorType.
     */
    virtual void copyToFloatArray(const DescriptorType &p, float * out) const
    {
      for (int i = 0; i < nr_dimensions_; ++i)
        out[i] = p.histogram[i];
    }
  };
}

/**
 * @brief A data structure that contains the parameters for pcl::SpinImageEstimation.
 */
struct spin_image_params
{
  unsigned int image_width; ///< image width of spin image.
  unsigned int min_pts_neighb; ///< minimum number of neighbours for spin-image generation.
  double support_angle_cos; ///< cosine of angle of spin image revolution.
  double radius; ///< radius of search fro the spin-image generation.
  unsigned int max_points; ///< Maximum number of points to compute spin image.
  spin_image_params() :
      image_width(8), support_angle_cos(0.5), min_pts_neighb(16), radius(0.03), max_points(500)
  {
  } ///< Default constructor.
};

/**
 * @brief Implementation of pcl::SpinImageEstimation.
 *
 * A class that implements the estimation of spin images.
 */
class SpinImage
{
private:
  unsigned int _image_width; ///< image width of spin image.
  unsigned int _min_pts_neighb; ///< minimum number of neighbours for spin-image generation.
  double _support_angle_cos; ///< cosine of angle of spin image revolution.
  double _radius; ///< radius of search for the spin-image generation.
  unsigned int _max_points; ///< Maximum number of points to compute spin image.
  VoxelGridFilter _filter; ///< downsampler
public:
  /**
   * Default class constructor
   */
  SpinImage();

  /**
   * Class constructor.
   * @param params a filled spin_image_params data structure.
   */
  SpinImage(struct spin_image_params params);

  /**
   * Class constructor.
   * @param image_width image width of spin image.
   * @param support_angle_cos cosine of angle of spin image revolution.
   * @param min_pts_neighb minimum number of neighbours for spin-image generation.
   * @param radius radius of search for the spin-image generation.
   * @param max_points Maximum number of points to compute spin image
   */
  SpinImage(unsigned int image_width, double support_angle_cos, unsigned int min_pts_neighb, double radius,
            unsigned int max_points);

  /**
   * Set image width
   * @param image_width image width of spin image.
   */
  void set_image_width(int image_width);

  /**
   * Set cosine of angle of spin-image revolution.
   * @param support_angle_cos cosine of angle of spin image revolution
   */
  void set_support_angle_cos(double support_angle_cos);

  /**
   * Set minimum neighbours for spin-image generation.
   * @param min_pts_neighb minimum number of neighbours for spin-image generation.
   */
  void set_min_points_neighbour(int min_pts_neighb);

  /**
   * Set radius for search of spin-image generation.
   * @param radius radius of search for the spin-image generation.
   */
  void set_radius(double radius);

  /**
   * Compute the spin image
   * @param cloud pointer to points which spin-image wants to be computer
   * @return Point cloud of spin-images.
   */
  DescriptorCloud compute_spin_image(const PointCloud::Ptr & cloud);

  /**
   * Class destructor.
   */
  ~SpinImage()
  {
  }
  ;
};

/**
 * @brief Shared pointer to SpinImage class
 */
typedef boost::shared_ptr<SpinImage> DescriptorPtr;

#endif /* _SPIN_IMAGE_H_ */
