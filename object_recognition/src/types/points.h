/** @file points.h
 *  @brief File with Point types used throughout this project.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#ifndef _TYPES_POINTS_H_
#define _TYPES_POINTS_H_

/**
 * @brief PointType definition used in the project
 */
typedef pcl::PointXYZ PointType;

/**
 * @brief PointCloud definition used in the project
 */
typedef pcl::PointCloud<PointType> PointCloud;



#endif /* _TYPES_POINTS_H_ */
