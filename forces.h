#pragma once
/*
This header file contains common functions for modelling forces in dual quaternion space
*/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_base.h"

namespace Dynamics
{

template<typename T>
void Check_ERROR_DUALQUATPURE(const std::string& error, const dualquat::DualQuaternion<T> dq, const T& tol)
{
    if(!dualquat::is_pure(dq,tol))
    {
        throw std::invalid_argument(("\n Not pure imaginary dual quaternion at : " + error));
    }
}

/*
Generalized wrench given a force and a moment
*/
template<typename T>
dualquat::DualQuaternion<T> 
Wrench(const Eigen::Vector3<T>& force, const Eigen::Vector3<T>& moment)
{
    return dualquat::DualQuaternion(force,moment);
}

/*
Generalized wrench given a force and a moment
*/
template<typename T>
dualquat::DualQuaternion<T> 
Wrench(const Eigen::Quaternion<T>& force, const Eigen::Quaternion<T>& moment)
{
    Check_ERROR_DUALQUATPURE("Wrench()",dualquat::DualQuaternion(force,moment),T(0.0001));
    return dualquat::DualQuaternion(force,moment);
}

/*
Generalized wrench at some point given a force and a moment
*/
template<typename T>
dualquat::DualQuaternion<T> 
Wrench(const Eigen::Vector3<T>& force,  const Eigen::Vector3<T>& moment, const Eigen::Vector3<T>& point)
{
    return dualquat::DualQuaternion(force,(moment+point.cross(force)));
}


} // namespace Dynamics
