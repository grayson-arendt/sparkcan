/**
 * @file SparkFlex.hpp
 * @brief Source file for the SPARK Flex motor controller interface
 * @author Grayson Arendt
 */

#pragma once

#include "SparkBase.hpp"

class SparkFlex : public SparkBase
{
public:
    explicit SparkFlex(const std::string &interfaceName, uint8_t deviceId)
        : SparkBase(interfaceName, deviceId) {}
};
