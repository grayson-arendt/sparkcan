/**
 * @file SparkMax.hpp
 * @brief Source file for the SPARK MAX motor controller interface
 * @author Grayson Arendt
 */

#pragma once

#include "SparkBase.hpp"

/**
 * @class SparkMax
 * @brief A C++ interface for controlling REV Robotics SPARK MAX via CAN bus
 *
 * This class provides methods to configure, control, and monitor SPARK motor controllers.
 * It supports various control modes, parameter settings, and status readings.
 */
class SparkMax : public SparkBase
{
public:
    explicit SparkMax(const std::string &interfaceName, uint8_t deviceId)
        : SparkBase(interfaceName, deviceId) {}
};
