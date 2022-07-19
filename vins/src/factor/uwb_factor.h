/*******************************************************
 * Copyright (C) 2021, Urban Robotics Lab, Korea Advanced Institute of Science and Technology (KAIST)
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <typeinfo>

#include "../estimator/parameters.h"
#include "../utility/utility.h"

template <typename T>
void showParameterInfo(const T x)
{
  std::cout << "Size of parameter : " << sizeof(x) << std::endl;
  std::cout << "Type of parameter : " << typeid(x).name() << std::endl;
  std::cout << "Value of parameter : " << x << std::endl;
}

struct UWBFactor
{
  UWBFactor(Eigen::Vector2d P) : anchor_pose(P){}

  template <typename T>
  bool operator()(const T* const range, const T* pose, T* residual) const
  {
    const T x_diff = anchor_pose[0] - pose[0];
    const T y_diff = anchor_pose[1] - pose[1];
    const double one = 1;
    const T measurement_range = sqrt(pow(x_diff, 2) + pow(y_diff, 2)) + one;

    // To find parameter values!!
    // showParameterInfo(x_diff);
    // showParameterInfo(y_diff);
    // showParameterInfo(pose);
    // showParameterInfo(range);
    // showParameterInfo(measurement_range);

    if(!USE_METHOD)
    {
      //std::cout << "Origin Method of UWB Cost Function !" << std::endl;
      residual[0] = range[0] - sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    }

    // Using Cross Entropy in Mutual Information Theory
    // residual = -ylog(x), where y is GT, x is measurement
    // if(USE_METHOD)
    // {
    //   std::cout << "Mutual Information Method of UWB Cost Function !" << std::endl;
    //   residual[0] = - range[0] * log2(measurement_range);
    // }


    return true;
  }

  void showPose()
  {
    std::cout << "anchor position x : " << anchor_pose[0] << std::endl;
    std::cout << "anchor position y : " << anchor_pose[1] << std::endl;
  }

private:
  Eigen::Vector2d anchor_pose;
};
