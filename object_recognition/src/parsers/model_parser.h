/** @file model_parser.h
 *  @brief Parsers implementation for reading different classes of images
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <string>
#include <vector>
#include <utility>

#include <iostream>
#include <fstream>
#include "../util/category.h"
#include "training_files_parser.h"


#ifndef _MODEL_PARSER_H_
#define _MODEL_PARSER_H_

static const std::string DUCK_CLASS = "duck";  ///< @brief Key for duck class
static const std::string HUMAN_CLASS = "human";  ///< @brief Key for human parameter
static const std::string UNKNOWN_CLASS = "unknown";  ///< @brief Key for unknown parameter

/**
 * Parser for known models
 * @param file_name file with models
 * @return filled categories
 */
std::vector<Category> get_models(const std::string & file_name);


#endif /* _MODEL_PARSER_H_ */
