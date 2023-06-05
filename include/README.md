TODO: update README

# Dual_Quaternion_Forward_Kinematics
C++ Library for performing robot dynamic modelling using dual quaternion algebra.

Dual quaternions are extentions of quaternions as a dual number, just as quaternions of unit length can be used to represent 3D rotations, dual quaternions of unit length can be used to represent 3D rigid motions. Dual quaternions provide a more stable and compact form for representing rigid body motion, and a unified space for performing modelling, control and planning when compared to classical tranformation methods.

This library should help you model the dynamics of your robot using only dual quaternions. The underlying assumption is that you know the DH parameters, and the inertial matrix of each joint on your robot.

## Dependencies

1. [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [dualquat](https://github.com/Hasenpfote/dualquat)

## Usage

## Wrenches

## Twists

## Dual acceleration

## Mapping joint rates to pose

## Dimensional analysis

## Notes

## Compatibility

Supports C++ 11 or higher.

| Compiler | Version           | Remarks |
| -------- | ----------------- | ------- |
| gcc      | 5.5.0 or higher.  |         |
| clang    | 7.0.0 or higher.  |         |
| msvc     | 16.5.4 or higher. |         |

## Refrences

