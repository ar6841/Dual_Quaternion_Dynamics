#pragma once
/*
This header file contains common functions for creating the dual intertia matrix, dual momentum vector
*/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_base.h"

namespace Dynamics
{

template<typename T>
Eigen::Matrix<T,8,8> // TODO: Make this a sparse matrix
DualInertia(const T& mass, const Eigen::Matrix<T,3,3>& inertia_3x3)
{
    T zero = T(0);
    return (Eigen::Matrix<T,8,8>()<<T(1),zero,zero,zero,zero,zero,zero,zero
                                    zero, mass, zero, zero, zero, zero, zero, zero
                                    zero, zero, mass, zero, zero, zero, zero, zero
                                    zero, zero, zero, mass, zero, zero, zero, zero
                                    zero, zero, zero, zero, T(1), zero, zero, zero
                                    zero, zero, zero, zero, zero, inertia(0,0), inertia(0,1), inertia(0,2)
                                    zero, zero, zero, zero, zero, inertia(1,0), inertia(1,1), inertia(1,2)
                                    zero, zero, zero, zero, zero, inertia(2,0), inertia(2,1), inertia(2,2)).finished();
}

template<typename T>
Eigen::Matrix<T,8,1>
DualMomentum(const Eigen::Matrix<T,8,8>& DualIntertia, const dualquat::DualQuaternion<T>& twist)
{
    return (DualInertia*dualquat_swap(twist)).finished();
}

template<typename T>
Eigen::Matrix<T,8,1>
DualMomentum(const Eigen::Matrix<T,8,8>& DualIntertia, const Eigen::Matrix<T,8,1>& twist)
{
    return (DualInertia*(Eigen::Matrix<T,8,1>(twist(4,1),
                                            twist(5,1),
                                            twist(6,1),
                                            twist(7,1),
                                            twist(0,1),
                                            twist(1,1),
                                            twist(2,1),
                                            twist(3,1)).finished())).finished();
}


} // namespace Dynamics