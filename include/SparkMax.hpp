/**
 * @file SparkMax.hpp
 * @brief Source file for the SPARK MAX motor controller interface
 * @author Grayson Arendt
 */

#pragma once

#include "SparkBase.hpp"

class SparkMax : public SparkBase
{
public:
    explicit SparkMax(const std::string &interfaceName, uint8_t deviceId)
        : SparkBase(interfaceName, deviceId) {}
};
