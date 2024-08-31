/**
 * @file SparkBase.hpp
 * @brief Header file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#ifndef SPARKBASE_HPP
#define SPARKBASE_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <thread>
#include <variant>
#include <system_error>

/// Coloring error messages
#define RED "\033[31m"
#define RESET "\033[0m"

/// Parameter type for unsigned integers
constexpr uint8_t PARAM_TYPE_UINT = 0x01;

/// Parameter type for floating-point numbers
constexpr uint8_t PARAM_TYPE_FLOAT = 0x02;

/// Parameter type for boolean values
constexpr uint8_t PARAM_TYPE_BOOL = 0x03;

/**
 * @brief System control commands for the SPARK controller
 */
enum class SystemControl : uint32_t
{
    BurnFlash = 0x205FC80,        /**< Burn the current configuration to flash memory */
    FactoryDefaults = 0x2051D00,  /**< Restore the controller to factory default settings */
    FactoryReset = 0x2051D40,     /**< Perform a factory reset on the controller */
    Identify = 0x2051D80,         /**< Trigger the controller to identify itself */
    ResetFaults = 0x2053000,      /**< Reset all faults on the controller */
    ClearStickyFaults = 0x2054400 /**< Clear sticky faults on the controller */
};

/**
 * @brief Motor control commands for the SPARK controller
 */
enum class MotorControl : uint32_t
{
    AppliedOutput = 0x2050080, /**< Command to set the motor's applied output */
    Velocity = 0x2050480,      /**< Command to set the motor's velocity */
    SmartVelocity = 0x20504C0, /**< Command to set the motor's smart velocity */
    Position = 0x2050C80,      /**< Command to set the motor's position */
    Voltage = 0x2051080,       /**< Command to set the motor's voltage */
    Current = 0x20510C0,       /**< Command to set the motor's current */
    SmartMotion = 0x2051480    /**< Command to set the motor's smart motion */
};

/**
 * @brief Status periods for the SPARK controller
 */
enum class Status : uint32_t
{
    Period0 = 0x2051800, /**< Status period 0 */
    Period1 = 0x2051840, /**< Status period 1 */
    Period2 = 0x2051880, /**< Status period 2 */
    Period3 = 0x20518C0, /**< Status period 3 */
    Period4 = 0x2051900  /**< Status period 4 */
};

/**
 * @brief Parameters for the SPARK controller
 */
enum class Parameter : uint32_t
{
    kCanID = 0,
    kInputMode = 1,
    kMotorType = 2,
    kCommAdvance = 3,
    kSensorType = 4,
    kCtrlType = 5,
    kIdleMode = 6,
    kInputDeadband = 7,
    kFeedbackSensorPID0 = 8,
    kFeedbackSensorPID1 = 9,
    kPolePairs = 10,
    kCurrentChop = 11,
    kCurrentChopCycles = 12,
    kP_0 = 13,
    kI_0 = 14,
    kD_0 = 15,
    kF_0 = 16,
    kIZone_0 = 17,
    kDFilter_0 = 18,
    kOutputMin_0 = 19,
    kOutputMax_0 = 20,
    kP_1 = 21,
    kI_1 = 22,
    kD_1 = 23,
    kF_1 = 24,
    kIZone_1 = 25,
    kDFilter_1 = 26,
    kOutputMin_1 = 27,
    kOutputMax_1 = 28,
    kP_2 = 29,
    kI_2 = 30,
    kD_2 = 31,
    kF_2 = 32,
    kIZone_2 = 33,
    kDFilter_2 = 34,
    kOutputMin_2 = 35,
    kOutputMax_2 = 36,
    kP_3 = 37,
    kI_3 = 38,
    kD_3 = 39,
    kF_3 = 40,
    kIZone_3 = 41,
    kDFilter_3 = 42,
    kOutputMin_3 = 43,
    kOutputMax_3 = 44,
    kInverted = 45,
    kOutputRatio = 46,
    kSerialNumberLow = 47,
    kSerialNumberMid = 48,
    kSerialNumberHigh = 49,
    kLimitSwitchFwdPolarity = 50,
    kLimitSwitchRevPolarity = 51,
    kHardLimitFwdEn = 52,
    kHardLimitRevEn = 53,
    kSoftLimitFwdEn = 54,
    kSoftLimitRevEn = 55,
    kRampRate = 56,
    kFollowerID = 57,
    kFollowerConfig = 58,
    kSmartCurrentStallLimit = 59,
    kSmartCurrentFreeLimit = 60,
    kSmartCurrentConfig = 61,
    kMotorKv = 63,
    kMotorR = 64,
    kMotorL = 65,
    kEncoderCountsPerRev = 69,
    kEncoderAverageDepth = 70,
    kEncoderSampleDelta = 71,
    kEncoderInverted = 72,
    kClosedLoopVoltageMode = 74,
    kCompensatedNominalVoltage = 75,
    kSmartMotionMaxVelocity_0 = 76,
    kSmartMotionMaxAccel_0 = 77,
    kSmartMotionMinVelOutput_0 = 78,
    kSmartMotionAllowedClosedLoopError_0 = 79,
    kSmartMotionAccelStrategy_0 = 80,
    kSmartMotionMaxVelocity_1 = 81,
    kSmartMotionMaxAccel_1 = 82,
    kSmartMotionMinVelOutput_1 = 83,
    kSmartMotionAllowedClosedLoopError_1 = 84,
    kSmartMotionAccelStrategy_1 = 85,
    kSmartMotionMaxVelocity_2 = 86,
    kSmartMotionMaxAccel_2 = 87,
    kSmartMotionMinVelOutput_2 = 88,
    kSmartMotionAllowedClosedLoopError_2 = 89,
    kSmartMotionAccelStrategy_2 = 90,
    kSmartMotionMaxVelocity_3 = 91,
    kSmartMotionMaxAccel_3 = 92,
    kSmartMotionMinVelOutput_3 = 93,
    kSmartMotionAllowedClosedLoopError_3 = 94,
    kSmartMotionAccelStrategy_3 = 95,
    kIMaxAccum_0 = 96,
    kSlot3Placeholder1_0 = 97,
    kSlot3Placeholder2_0 = 98,
    kSlot3Placeholder3_0 = 99,
    kIMaxAccum_1 = 100,
    kSlot3Placeholder1_1 = 101,
    kSlot3Placeholder2_1 = 102,
    kSlot3Placeholder3_1 = 103,
    kIMaxAccum_2 = 104,
    kSlot3Placeholder1_2 = 105,
    kSlot3Placeholder2_2 = 106,
    kSlot3Placeholder3_2 = 107,
    kIMaxAccum_3 = 108,
    kSlot3Placeholder1_3 = 109,
    kSlot3Placeholder2_3 = 110,
    kSlot3Placeholder3_3 = 111,
    kPositionConversionFactor = 112,
    kVelocityConversionFactor = 113,
    kClosedLoopRampRate = 114,
    kSoftLimitFwd = 115,
    kSoftLimitRev = 116,
    kAnalogPositionConversion = 119,
    kAnalogVelocityConversion = 120,
    kAnalogAverageDepth = 121,
    kAnalogSensorMode = 122,
    kAnalogInverted = 123,
    kAnalogSampleDelta = 124,
    kDataPortConfig = 127,
    kAltEncoderCountsPerRev = 128,
    kAltEncoderAverageDepth = 129,
    kAltEncoderSampleDelta = 130,
    kAltEncoderInverted = 131,
    kAltEncoderPositionFactor = 132,
    kAltEncoderVelocityFactor = 133,
    kHallSensorSampleRate = 136,
    kHallSensorAverageDepth = 137,
    kDutyCyclePositionFactor = 139,
    kDutyCycleVelocityFactor = 140,
    kDutyCycleInverted = 141,
    kDutyCycleAverageDepth = 143,
    kPositionPIDWrapEnable = 149,
    kPositionPIDMinInput = 150,
    kPositionPIDMaxInput = 151,
    kDutyCyclePrescalar = 153,
    kDutyCycleZeroOffset = 154,
};

/**
 * @class SparkBase
 * @brief A base class for controlling REV Robotics SPARK motor controllers via CAN bus
 *
 * This class provides methods to configure, control, and monitor SPARK motor controllers.
 * It supports various control modes, parameter settings, and status readings.
 */
class SparkBase
{
private:
    int soc;
    uint8_t deviceId;
    struct sockaddr_can addr;
    struct ifreq ifr;

    mutable std::map<Status,
                     std::pair<uint64_t, std::chrono::steady_clock::time_point>>
        cachedStatus;

    void SendCanFrame(
        uint32_t arbitrationId, uint8_t dlc,
        const std::array<uint8_t, 8> &data = std::array<uint8_t, 8>{}) const;

    void SendControlMessage(std::variant<MotorControl, SystemControl> command,
                            float value);

    uint64_t ReadPeriodicStatus(Status period) const;

    void SetParameter(
        Parameter parameterId, uint8_t parameterType,
        std::variant<float, uint32_t, uint16_t, uint8_t, bool> value);

public:
    /**
     * @brief Initializes SparkBase with the specified CAN interface and ID
     *
     * @param interfaceName The name of the CAN interface (e.g., "can0")
     * @param deviceId The CAN ID of the SPARK controller controller (0-62)
     * @throws std::invalid_argument if deviceId is greater than 62
     * @throws std::runtime_error if socket creation, IOCTL, or binding fails
     */
    SparkBase(const std::string &interfaceName, uint8_t deviceId);

    /**
     * @brief Destructor for SparkBase
     *
     * Closes the socket connection
     */
    virtual ~SparkBase();

    // SystemControl Methods //

    /**
     * @brief Sends a heartbeat signal to keep the SPARK controller active
     */
    void Heartbeat();

    /**
     * @brief Resets all faults on the SPARK controller
     */
    void ResetFaults();

    /**
     * @brief Clears sticky faults on the SPARK controller
     */
    void ClearStickyFaults();

    /**
     * @brief Burns the current configuration to the SPARK controller's flash memory
     */
    void BurnFlash();

    /**
     * @brief Resets the SPARK controller to factory default settings
     */
    void FactoryDefaults();

    /**
     * @brief Performs a factory reset on the SPARK controller
     */
    void FactoryReset();

    /**
     * @brief Triggers the SPARK controller to identify itself
     */
    void Identify();

    // MotorControl Methods //

    /**
     * @brief Sets the motor's applied output
     * @param appliedOutput The desired applied output, range: [-1.0, 1.0]
     */
    void SetAppliedOutput(float appliedOutput);

    /**
     * @brief Sets the motor's velocity
     * @param velocity The desired velocity
     */
    void SetVelocity(float velocity);

    /**
     * @brief Sets the motor's smart velocity
     * @param smartVelocity The desired smart velocity
     */
    void SetSmartVelocity(float smartVelocity);

    /**
     * @brief Sets the motor's position
     * @param position The desired position
     */
    void SetPosition(float position);

    /**
     * @brief Sets the motor's voltage
     * @param voltage The desired voltage
     */
    void SetVoltage(float voltage);

    /**
     * @brief Sets the motor's current
     * @param current The desired current
     */
    void SetCurrent(float current);

    /**
     * @brief Sets the motor's smart motion
     * @param smartMotion The desired smart motion value
     */
    void SetSmartMotion(float smartMotion);

    // Status Methods //

    /**
     * @brief Retrieves the current applied output
     * @return float The applied output, range: [-1.0, 1.0]
     */
    float GetAppliedOutput() const;

    /**
     * @brief Gets the current velocity
     * @return float The current velocity (RPM)
     */
    float GetVelocity() const;

    /**
     * @brief Gets the current temperature of the SPARK controller
     * @return float The temperature in degrees Celsius
     */
    float GetTemperature() const;

    /**
     * @brief Gets the current voltage of the SPARK controller
     * @return float The voltage in volts
     */
    float GetVoltage() const;

    /**
     * @brief Gets the current drawn by the SPARK controller
     * @return float The current in amperes
     */
    float GetCurrent() const;

    /**
     * @brief Gets the current position of the motor
     * @return float The position (ticks)
     */
    float GetPosition() const;

    /**
     * @brief Gets the current analog voltage
     * @return float The analog voltage in volts
     */
    float GetAnalogVoltage() const;

    /**
     * @brief Gets the current analog velocity
     * @return float The analog velocity (RPM)
     */
    float GetAnalogVelocity() const;

    /**
     * @brief Gets the current analog position
     * @return float The analog position (ticks)
     */
    float GetAnalogPosition() const;

    /**
     * @brief Gets the velocity from the alternate encoder
     * @return float The alternate encoder velocity (RPM)
     */
    float GetAlternateEncoderVelocity() const;

    /**
     * @brief Gets the position from the alternate encoder
     * @return float The alternate encoder position (ticks)
     */
    float GetAlternateEncoderPosition() const;

    /**
     * @brief Retrieves the current faults
     * @return uint16_t A bitfield representing the current faults
     */
    uint16_t GetFaults() const;

    /**
     * @brief Retrieves the sticky faults
     * @return uint16_t A bitfield representing the sticky faults
     */
    uint16_t GetStickyFaults() const;

    /**
     * @brief Gets the invert, brake, and follower status
     * @return uint8_t A bitfield representing the invert, brake, and follower status
     */
    uint8_t GetInvertBrakeIsFollower() const;

    /**
     * @brief Checks if the motor is inverted
     * @return bool True if the motor is inverted, false otherwise
     */
    bool GetInverted() const;

    /**
     * @brief Gets the current idle mode
     * @return bool True if in brake mode, false if in coast mode
     */
    bool GetIdleMode() const;

    /**
     * @brief Checks if the SPARK controller is in follower mode
     * @return bool True if in follower mode, false otherwise
     */
    bool IsFollower() const;

    // Parameters //

    // Basic //

    /**
     * @brief Sets the input mode
     * @param mode The input mode
     */
    void SetInputMode(uint8_t mode);

    /**
     * @brief Sets the motor type
     * @param type 0 for Brushed, 1 for Brushless
     * @throws std::invalid_argument if type is not 0 or 1
     */
    void SetMotorType(uint8_t type);

    /**
     * @brief Sets the sensor type
     * @param sensor 0 for No Sensor, 1 for Hall Sensor, 2 for Encoder
     * @throws std::invalid_argument if sensor is not 0, 1, or 2
     */
    void SetSensorType(uint8_t sensor);

    /**
     * @brief Sets the idle mode
     * @param mode 0 for Coast, 1 for Brake
     * @throws std::invalid_argument if mode is not 0 or 1
     */
    void SetIdleMode(uint8_t mode);

    /**
     * @brief Sets the input deadband
     * @param deadband The deadband value
     */
    void SetInputDeadband(float deadband);

    /**
     * @brief Sets whether the motor is inverted
     * @param inverted True to invert the motor, false otherwise
     */
    void SetInverted(bool inverted);

    /**
     * @brief Sets the ramp rate
     * @param rate The ramp rate in seconds from neutral to full output
     */
    void SetRampRate(float rate);

    // Advanced //

    /**
     * @brief Sets the commutation advance
     * @param advance The commutation advance value
     */
    void SetCommAdvance(float advance);

    /**
     * @brief Sets the motor Kv (velocity constant)
     * @param kv The Kv value
     */
    void SetMotorKv(uint16_t kv);

    /**
     * @brief Sets the motor resistance
     * @param r The resistance value
     */
    void SetMotorR(uint16_t r);

    /**
     * @brief Sets the motor inductance
     * @param l The inductance value
     */
    void SetMotorL(uint16_t l);

    // Closed Loop //

    /**
     * @brief Sets the control type
     * @param type 0 for Duty Cycle, 1 for Velocity, 2 for Voltage, 3 for Position
     * @throws std::invalid_argument if type is not 0, 1, 2, or 3
     */
    void SetCtrlType(uint8_t type);

    /**
     * @brief Sets the feedback sensor for PID0
     * @param sensor The sensor type
     */
    void SetFeedbackSensorPID0(uint16_t sensor);

    /**
     * @brief Sets the closed loop voltage mode
     * @param mode 0 for Disabled, 1 for Control Loop Voltage Output Mode, 2 for Voltage Compensation Mode
     * @throws std::invalid_argument if mode is not 0, 1, or 2
     */
    void SetClosedLoopVoltageMode(uint8_t mode);

    /**
     * @brief Sets the compensated nominal voltage
     * @param voltage The nominal voltage to compensate for
     */
    void SetCompensatedNominalVoltage(float voltage);

    /**
     * @brief Enables or disables position PID wrap
     * @param enable True to enable, false to disable
     */
    void SetPositionPIDWrapEnable(bool enable);

    /**
     * @brief Sets the minimum input for position PID
     * @param minInput The minimum input value
     */
    void SetPositionPIDMinInput(float minInput);

    /**
     * @brief Sets the maximum input for position PID
     * @param maxInput The maximum input value
     */
    void SetPositionPIDMaxInput(float maxInput);

    // Brushless //

    /**
     * @brief Sets the number of pole pairs for brushless motors
     * @param pairs The number of pole pairs
     */
    void SetPolePairs(uint16_t pairs);

    // Current Limit //

    /**
     * @brief Sets the current chop limit
     * @param chop The current chop limit (0-125 amps)
     * @throws std::invalid_argument if chop is greater than 125
     */
    void SetCurrentChop(float chop);

    /**
     * @brief Sets the number of cycles for current chopping
     * @param cycles The number of cycles
     */
    void SetCurrentChopCycles(uint16_t cycles);

    /**
     * @brief Sets the smart current stall limit
     * @param limit The stall current limit
     */
    void SetSmartCurrentStallLimit(uint16_t limit);

    /**
     * @brief Sets the smart current free limit
     * @param limit The free current limit
     */
    void SetSmartCurrentFreeLimit(uint16_t limit);

    /**
     * @brief Sets the smart current configuration
     * @param config The configuration value
     */
    void SetSmartCurrentConfig(uint16_t config);

    // PIDF //

    /**
     * @brief Sets the proportional gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param p The proportional gain value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetP(uint8_t slot, float p);

    /**
     * @brief Sets the integral gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param i The integral gain value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetI(uint8_t slot, float i);

    /**
     * @brief Sets the derivative gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param d The derivative gain value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetD(uint8_t slot, float d);

    /**
     * @brief Sets the feedforward gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param f The feedforward gain value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetF(uint8_t slot, float f);

    /**
     * @brief Sets the integral zone for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param iZone The integral zone value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetIZone(uint8_t slot, float iZone);

    /**
     * @brief Sets the derivative filter for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param dFilter The derivative filter value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetDFilter(uint8_t slot, float dFilter);

    /**
     * @brief Sets the minimum output for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param min The minimum output value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetOutputMin(uint8_t slot, float min);

    /**
     * @brief Sets the maximum output for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param max The maximum output value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetOutputMax(uint8_t slot, float max);

    // Limits //

    /**
     * @brief Enables or disables the forward hard limit switch
     * @param enable True to enable, false to disable
     */
    void SetHardLimitFwdEn(bool enable);

    /**
     * @brief Enables or disables the reverse hard limit switch
     * @param enable True to enable, false to disable
     */
    void SetHardLimitRevEn(bool enable);

    /**
     * @brief Sets the polarity of the forward limit switch
     * @param polarity True for normally open, false for normally closed
     */
    void SetLimitSwitchFwdPolarity(bool polarity);

    /**
     * @brief Sets the polarity of the reverse limit switch
     * @param polarity True for normally open, false for normally closed
     */
    void SetLimitSwitchRevPolarity(bool polarity);

    /**
     * @brief Enables or disables the forward soft limit
     * @param enable True to enable, false to disable
     */
    void SetSoftLimitFwdEn(bool enable);

    /**
     * @brief Enables or disables the reverse soft limit
     * @param enable True to enable, false to disable
     */
    void SetSoftLimitRevEn(bool enable);

    /**
     * @brief Sets the forward soft limit
     * @param limit The forward soft limit value
     */
    void SetSoftLimitFwd(float limit);

    /**
     * @brief Sets the reverse soft limit
     * @param limit The reverse soft limit value
     */
    void SetSoftLimitRev(float limit);

    // Follower //

    /**
     * @brief Sets the follower ID for this SparkBase
     * @param id The CAN ID of the SPARK controller to follow
     */
    void SetFollowerID(uint32_t id);

    /**
     * @brief Sets the follower configuration
     * @param config The follower configuration value
     */
    void SetFollowerConfig(uint32_t config);

    // Encoder Port //

    /**
     * @brief Sets the encoder counts per revolution
     * @param counts The number of counts per revolution
     */
    void SetEncoderCountsPerRev(uint16_t counts);

    /**
     * @brief Sets the encoder average depth
     * @param depth The average depth (1-64)
     * @throws std::invalid_argument if depth is 0 or greater than 64
     */
    void SetEncoderAverageDepth(uint8_t depth);

    /**
     * @brief Sets the encoder sample delta
     * @param delta The sample delta (1-255)
     * @throws std::invalid_argument if delta is 0
     */
    void SetEncoderSampleDelta(uint8_t delta);

    /**
     * @brief Sets whether the encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetEncoderInverted(bool inverted);

    /**
     * @brief Sets the position conversion factor
     * @param factor The position conversion factor
     */
    void SetPositionConversionFactor(float factor);

    /**
     * @brief Sets the velocity conversion factor
     * @param factor The velocity conversion factor
     */
    void SetVelocityConversionFactor(float factor);

    /**
     * @brief Sets the closed loop ramp rate
     * @param rampRate The ramp rate in seconds from neutral to full output
     */
    void SetClosedLoopRampRate(float rampRate);

    /**
     * @brief Sets the hall sensor sample rate
     * @param rate The sample rate in Hz
     */
    void SetHallSensorSampleRate(float rate);

    /**
     * @brief Sets the hall sensor average depth
     * @param depth The average depth
     */
    void SetHallSensorAverageDepth(uint16_t depth);

    // Smart Motion //

    /**
     * @brief Sets the maximum velocity for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param velocity The maximum velocity
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMaxVelocity(uint8_t slot, float velocity);

    /**
     * @brief Sets the maximum acceleration for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param accel The maximum acceleration
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMaxAccel(uint8_t slot, float accel);

    /**
     * @brief Sets the minimum velocity output for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param minVel The minimum velocity output
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMinVelOutput(uint8_t slot, float minVel);

    /**
     * @brief Sets the allowed closed loop error for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param error The allowed closed loop error
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error);

    /**
     * @brief Sets the acceleration strategy for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param strategy The acceleration strategy
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionAccelStrategy(uint8_t slot, float strategy);

    /**
     * @brief Sets the maximum accumulator value for the I term in the specified slot
     * @param slot The PID slot (0-3)
     * @param maxAccum The maximum accumulator value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetIMaxAccum(uint8_t slot, float maxAccum);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder1(uint8_t slot, float value);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder2(uint8_t slot, float value);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder3(uint8_t slot, float value);

    // Analog Sensor //

    /**
     * @brief Sets the conversion factor for analog position readings
     * @param factor The conversion factor to apply to raw analog readings
     */
    void SetAnalogPositionConversion(float factor);

    /**
     * @brief Sets the conversion factor for analog velocity readings
     * @param factor The conversion factor to apply to raw analog readings
     */
    void SetAnalogVelocityConversion(float factor);

    /**
     * @brief Sets the average depth for analog readings
     * @param depth The average depth
     */
    void SetAnalogAverageDepth(uint16_t depth);

    /**
     * @brief Sets the analog sensor mode
     * @param mode 0 for Absolute, 1 for Relative
     * @throws std::invalid_argument if mode is not 0 or 1
     */
    void SetAnalogSensorMode(uint8_t mode);

    /**
     * @brief Sets whether the analog sensor is inverted
     * @param inverted True to invert the sensor, false otherwise
     */
    void SetAnalogInverted(bool inverted);

    /**
     * @brief Sets the sample delta for analog readings
     * @param delta The sample delta
     */
    void SetAnalogSampleDelta(uint16_t delta);

    // Alternate Encoder  //

    /**
     * @brief Configures the data port
     * @param config 0 for Default, 1 for Alternate Encoder Mode
     * @throws std::invalid_argument if config is not 0 or 1
     */
    void SetDataPortConfig(uint8_t config);

    /**
     * @brief Sets the counts per revolution for the alternate encoder
     * @param counts The number of counts per revolution
     */
    void SetAltEncoderCountsPerRev(uint16_t counts);

    /**
     * @brief Sets the average depth for the alternate encoder
     * @param depth The average depth (1-64)
     * @throws std::invalid_argument if depth is 0 or greater than 64
     */
    void SetAltEncoderAverageDepth(uint8_t depth);

    /**
     * @brief Sets the sample delta for the alternate encoder
     * @param delta The sample delta (1-255)
     * @throws std::invalid_argument if delta is 0
     */
    void SetAltEncoderSampleDelta(uint8_t delta);

    /**
     * @brief Sets whether the alternate encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetAltEncoderInverted(bool inverted);

    /**
     * @brief Sets the position factor for the alternate encoder
     * @param factor The position factor
     */
    void SetAltEncoderPositionFactor(float factor);

    /**
     * @brief Sets the velocity factor for the alternate encoder
     * @param factor The velocity factor
     */
    void SetAltEncoderVelocityFactor(float factor);

    // Duty Cycle Absolute Encoder //

    /**
     * @brief Sets the position factor for the duty cycle encoder
     * @param factor The position factor
     */
    void SetDutyCyclePositionFactor(float factor);

    /**
     * @brief Sets the velocity factor for the duty cycle encoder
     * @param factor The velocity factor
     */
    void SetDutyCycleVelocityFactor(float factor);

    /**
     * @brief Sets whether the duty cycle encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetDutyCycleInverted(bool inverted);

    /**
     * @brief Sets the average depth for the duty cycle encoder
     * @param depth The average depth (0-7)
     * @throws std::invalid_argument if depth is greater than 7
     */
    void SetDutyCycleAverageDepth(uint8_t depth);

    /**
     * @brief Sets the prescalar for the duty cycle encoder
     * @param prescalar The prescalar value (0-71)
     * @throws std::invalid_argument if prescalar is greater than 71
     */
    void SetDutyCyclePrescalar(uint8_t prescalar);

    /**
     * @brief Sets the zero offset for the duty cycle encoder
     * @param offset The zero offset (0-1)
     * @throws std::invalid_argument if offset is greater than 1
     */
    void SetDutyCycleZeroOffset(float offset);
};

#endif // SPARKBASE_HPP
