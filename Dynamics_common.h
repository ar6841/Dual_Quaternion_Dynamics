#pragma once
/*
This header file contains common functions for creating the dual intertia matrix, 
*/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_base.h"

namespace Dynamics
{
template<typename T>
Eigen::Matrix<T,8,8> //TODO: Make this sparse matrix
DualInertia(const T& mass, const Eigen::Matrix<T,3,3>& inertia_3x3)
{
    T zero = T(0);
    return (Eigen::Matrix<T,8,8>()<<T(1),zero,zero,zero,zero,zero,zero,zero
                                    zero, mass, zero, zero, zero, zero, zero, zero
                                    zero, zero, mass, zero, zero, zero, zero, zero
                                    zero, zero, zero, mass, zero, zero, zero, zero
                                    zero, zero, zero, zero, T(1), zero, zero, zero
                                    zero, zero, zero, zero, zero, inertia(1,1), inertia(1,2), inertia(1,3)
                                    zero, zero, zero, zero, zero, inertia(2,1), inertia(2,2), inertia(3,3)
                                    zero, zero, zero, zero, zero, inertia(3,1), inertia(3,2), inertia(3,3)).finished();
}

template<typename T>
Eigen::Matrix<T,8,1>
DualMomentum(const Eigen::Matrix<T,8,8>& DualIntertia, const dualquat::DualQuaternion<T>& twist)
{
    return (DualInertia*dualquat_to_vec(twist)).finished();
}

template<typename T>
Eigen::Matrix<T,8,1>
DualMomentum(const Eigen::Matrix<T,8,8>& DualIntertia, const Eigen::Matrix<T,8,1>& twist)
{
    return (DualInertia*twist).finished();
}


} // namespace Dynamics