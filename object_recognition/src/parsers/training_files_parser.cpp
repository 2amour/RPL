/*
 * training_files_parser.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/parsers/training_files_parser.h"

std::vector<std::string> get_training_file_names(const std::string & path)
{
  std::vector<std::string> file_names;
  std::string file_name;

  std::ifstream infile(path.c_str());

  while (std::getline(infile, file_name))
  {
    file_names.push_back(file_name);
  }
  return file_names;
}


std::vector<FilePair> get_file_names(const std::string & path)
{
  std::vector<FilePair> file_names;
  std::ifstream infile(path.c_str());

  std::string file_key;
  std::string file_name;

  while (infile.good())
  {
    infile >> file_key;
    infile >> file_name;
    FilePair pair(file_key, file_name);
    file_names.push_back(pair);
  }
  return file_names;
}
