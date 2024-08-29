/**
 * @file SparkFlex.hpp
 * @brief Source file for the SPARK Flex motor controller interface
 * @author Grayson Arendt
 */

#pragma once

#include "SparkBase.hpp"

/**
 * @class SparkFlex
 * @brief A C++ interface for controlling REV Robotics SPARK Flex via CAN bus
 *
 * This class provides methods to configure, control, and monitor SPARK motor controllers.
 * It supports various control modes, parameter settings, and status readings.
 */
class SparkFlex : public SparkBase
{
public:
    explicit SparkFlex(const std::string &interfaceName, uint8_t deviceId)
        : SparkBase(interfaceName, deviceId) {}
};
