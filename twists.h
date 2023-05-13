#pragma once

/*
Header file to calculate the twist using forward pass from base frame to ith frame

NOTES: I have two choices here, to make a recursive forward pass function or to make a loop if I know how many links there are.
        I am going with the loop as I don't want to fill up the stack for large robots
*/

namespace Dynamics
{

} // namespace Dynamics 