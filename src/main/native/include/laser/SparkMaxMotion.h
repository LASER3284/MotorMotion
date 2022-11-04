/*
Copyright 2022 Camdenton LASER 3284

This file is part of MotorMotion.

MotorMotion is free software: you can redistribute it and/or modify it under 
the terms of the GNU Lesser General Public License as published by the Free 
Software Foundation, either version 3 of the License, or (at your option) any 
later version.

MotorMotion is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along 
with MotorMotion. If not, see <https://www.gnu.org/licenses/>. 
*/

/**
 * @file SparkMaxMotion.h
 * @brief 
 *      This file contains the declaration of the SparkMaxMotion class, which 
 *      implements MotorMotion for CAN Spark Max
 * @todo 
 *      Implement SparkMaxMotion
 */
#pragma once

#include <rev/CANSparkMax.h>
#include "laser/MotorMotion.h"
////////////////////////////////////////////////////////////////////////////////

namespace laser {

/**
 * @brief 
 *      This namespace contains the SparkMaxMotion class, which implements 
 *      MotorMotion
 */
namespace sparkmax {

    /**
     * @brief 
     *      This namespace is meant to contain defaults and constants for the 
     *      SparkMaxMotion class implementation
     * @todo 
     *      Fill in based on SparkMaxMotion implementation
     */
    namespace defaults {
        
    }

    /**
     * @class SparkMaxMotion SparkMaxMotion.h laser/SparkMaxMotion.h
     * @brief 
     *      This is the declaration of the SparkMaxMotion class, which
     *      implements MotorMotion
     * @todo 
     *      Inherit MotorMotion<rev::REVLibError, rev::CANSparkMax>
     */
    class SparkMaxMotion {
        
    };

} // namespace sparkmax

} // namespace laser