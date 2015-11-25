/*
 * spin_training.cpp
 *
 *  Created on: Nov 19, 2015
 *      Author: Sebastian Curi
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "descriptors/spin_image.h"
#include "parsers/ros_parameters_parser.h"
#include "parsers/training_files_parser.h"

#include "types/points.h"
#include "util/histogram_math.h"

const std::string NODE ="SPIN IMAGE TRAINING";

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage ./spin_training file_with_paths output_path" << std::endl;
    return -1;
  }
  std::string path = argv[1];
  std::string output_path = argv[2];

  DescriptorCloud histogram;
  std::vector<std::string> files = get_training_file_names(path);

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh;
  RosParameterParser rosparam_parser(nh);
  DescriptorPtr spin_image = rosparam_parser.get_descriptor();

  for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); ++it)
  {
    PointCloud cloud;
    if (pcl::io::loadPCDFile<PointType>(*it, cloud) == -1) //* load the file
    {
      std::cerr << "Couldn't read file " << *it << std::endl;
      return -1;
    }

    DescriptorCloud output = spin_image->compute_spin_image(cloud.makeShared());
    histogram += output;
    std::size_t found = it->rfind("/");
    if (found != std::string::npos)
    {
      std::string out_file = output_path + "spin_" + it->substr(1 + found);
      std::cout << out_file << std::endl;
      pcl::io::savePCDFileBinary(out_file, output);
    }
  }


  std::string out_file = output_path + path.substr(1 + path.rfind("/"), path.rfind(".") - path.rfind("/") - 1) + ".pcd";
  std::cout << out_file << std::endl;
  pcl::io::savePCDFileBinary(out_file, histogram);

  return 0;
}
