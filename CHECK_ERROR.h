#pragma once
/*
This header file contains common errors that must be checked
*/
#include "Eigen/Core"
#include "dualquat/dualquat_base.h"
namespace Errors
{

template<typename T>
void Check_DUALQUATPURE(const std::string& error, const dualquat::DualQuaternion<T> dq, const T& tol)
{
    if(!dualquat::is_pure(dq,tol))
    {
        throw std::invalid_argument(("\n Not pure imaginary dual quaternion at : " + error));
    }
}



} //Errors

