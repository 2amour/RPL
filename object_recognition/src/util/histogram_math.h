/** @file histogram_math.h
 *  @brief Functions for pcl::Histograms.
 *
 *  This file contains a class for implementing different functions that calculate parameters
 *  for pcl::Histograms
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */


#include <cmath>
#include "../descriptors/spin_image.h"

#ifndef _HISTOGRAM_MATH_H_
#define _HISTOGRAM_MATH_H_

/**
 * Get cross-correlation between two histograms
 * @param p pcl::Histogram
 * @param q pcl::Histogram
 * @return cross-correlation between p and q
 */
float get_correlation(DescriptorType p , DescriptorType q);

/**
 * Return similarity criterion based on cross-correlation and occlusion
 * @param p pcl::Histogram
 * @param q pcl::Histogram
 * @param lambda occlusion parameter
 * @return similarity between p and q
 */
float get_atan_similarity(DescriptorType p , DescriptorType q, double lambda);

/**
 * Return root-mean-squared error between p and q
 * @param p pcl::Histogram
 * @param q pcl::Histogram
 * @return root-mean-squared error between p and q
 */
float get_rms_error(DescriptorType p , DescriptorType q);

#endif /* _HISTOGRAM_MATH_H_ */
