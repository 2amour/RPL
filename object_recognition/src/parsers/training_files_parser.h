/** @file training_files_parser.h
 *  @brief Parsers implementations for training_files.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#ifndef _TRAINING_FILE_PARSER_H_
#define _TRAINING_FILE_PARSER_H_

/**
 * @brief File pair with key, new_file
 */
typedef std::pair<std::string, std::string> FilePair;

/**
 * Get vector of paths to models
 * @param path file with a list of paths to model
 * @return vector of paths to models
 */
std::vector<std::string> get_training_file_names(const std::string & path);

/**
 * Get file names from a path of files
 * @param path path_to_text that contains (key, files) entries
 * @return a vector of FilePair type with the read entries
 */
std::vector<FilePair> get_file_names(const std::string & path);
#endif /* _TRAINING_FILE_PARSER_H_ */
