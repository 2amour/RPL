/*
 * histogram_math.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/util/histogram_math.h"

float get_correlation(DescriptorType p, DescriptorType q)
{
  float sum_pq = 0;
  float sum_p = 0;
  float sum_q = 0;
  float sum_p2 = 0;
  float sum_q2 = 0;
  for (int i = 0; i < BIN_SIZE; ++i){
    float pi = p.histogram[i];
    float qi = q.histogram[i];

    sum_pq += pi * qi;
    sum_p  += pi;
    sum_p2 += pi * pi;
    sum_q  += qi;
    sum_q2 += qi * qi;
  }
  return (BIN_SIZE * sum_pq - sum_p*sum_q) / std::sqrt((BIN_SIZE*sum_p2 - sum_p*sum_p)*(BIN_SIZE*sum_q2 - sum_q*sum_q));
}

float get_atan_similarity(DescriptorType p, DescriptorType q, double lambda){
  return std::pow(atanh(get_correlation(p, q)), 2) - lambda/(BIN_SIZE-3);
}

float get_rms_error(DescriptorType p, DescriptorType q){
  float rms = 0.0;
  for (int i = 0; i < BIN_SIZE; ++i){
    float pi = p.histogram[i];
    float qi = q.histogram[i];

    rms += (pi - qi)*(pi - qi);
  }
  return std::sqrt(rms)/BIN_SIZE;
}
