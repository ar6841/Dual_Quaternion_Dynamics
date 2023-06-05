#pragma once
/*
This header file contains common functions for modelling forces in dual quaternion space
*/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_base.h"
#include "CHECK_ERROR.h"

namespace Dynamics
{
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
    Errors::Check_DUALQUATPURE("Wrench()",dualquat::DualQuaternion(force,moment),T(0.0001));
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
