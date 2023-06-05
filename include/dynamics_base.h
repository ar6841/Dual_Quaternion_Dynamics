#pragma once
/*
This header file contains common functions for creating the quaternionic inertia matrix
*/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_base.h"
#include <vector>

namespace Dynamics
{

/*
Function to take in a 3x3 inertial tensor and return the quaternionic tensor as a list
*/
template<typename T>
std::vector<Eigen::Quaternion<T>> //This is abf
QuaternionicInertia(const Eigen::Matrix<T,3,3>& inertia_tensor)
{
    return std::vector<Eigen::Quaternion<T>>{Eigen::Quaternion<T>(inertia_tensor(0,0),inertia_tensor(0,1),inertia_tensor(0,2)),
                        Eigen::Quaternion<T>(inertia_tensor(1,0),inertia_tensor(1,1),inertia_tensor(1,2)),
                        Eigen::Quaternion<T>(inertia_tensor(2,0),inertia_tensor(2,1),inertia_tensor(2,2))};
}



} // namespace Dynamics
