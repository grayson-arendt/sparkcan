/**
 * @file SparkBase.cpp
 * @brief Source file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#include "SparkBase.hpp"

SparkBase::SparkBase(const std::string &interfaceName, uint8_t deviceId)
    : deviceId(deviceId), soc(-1)
{
    if (deviceId > 62)
    {
        throw std::out_of_range(RED "Invalid CAN bus ID. Must be between 0 and 62." RESET);
    }

    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (soc < 0)
    {
        throw std::system_error(errno, std::generic_category(),
                                std::string(RED) + "Socket creation failed: " + strerror(errno) +
                                    "\nPossible causes:\n"
                                    "1. CAN modules not loaded (run 'sudo modprobe can' and 'sudo modprobe can_raw')\n"
                                    "2. System resource limitations\n" +
                                    std::string(RESET));
    }

    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interfaceName.c_str(), sizeof(ifr.ifr_name) - 1);
    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        close(soc);
        throw std::runtime_error(std::string(RED) + "IOCTL failed: " + std::string(strerror(errno)) +
                                 "\nPossible causes:\n"
                                 "1. CAN interface '" +
                                 interfaceName + "' does not exist\n"
                                                 "2. CAN interface is not up (run 'sudo ip link set " +
                                 interfaceName + " up')\n"
                                                 "3. CAN bus not initialized (run 'sudo ip link set " +
                                 interfaceName + " type can bitrate 1000000')\n" +
                                 std::string(RESET));
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(soc);
        throw std::runtime_error(std::string(RED) + "Binding to interface failed: " + std::string(strerror(errno)) +
                                 "\nPossible cause: Another program is already bound to this interface\n" +
                                 std::string(RESET));
    }
}

SparkBase::~SparkBase() { close(soc); }

void SparkBase::SendCanFrame(uint32_t deviceId, uint8_t dlc, const std::array<uint8_t, 8> &data) const
{
    struct can_frame frame = {};
    frame.can_id = deviceId | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data.data(), dlc);

    constexpr std::chrono::milliseconds WAIT_TIME(1);
    constexpr int MAX_ATTEMPTS = 1000;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt)
    {
        ssize_t bytesSent = write(soc, &frame, sizeof(frame));
        if (bytesSent == sizeof(frame))
        {
            return;
        }
        if (bytesSent < 0)
        {
            if (errno == ENOBUFS || errno == EAGAIN)
            {
                std::this_thread::sleep_for(WAIT_TIME);
                continue;
            }
            throw std::runtime_error(std::string(RED) + "Error sending CAN frame: " + std::string(strerror(errno)) + std::string(RESET));
        }
    }
    throw std::runtime_error(RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::SendControlMessage(std::variant<MotorControl, SystemControl> command, std::string commandName, float value, std::optional<float> minValue, std::optional<float> maxValue) const
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED + commandName + " must be a finite number." + RESET);
    }

    if (minValue && maxValue && (value < *minValue || value > *maxValue))
    {
        throw std::out_of_range(RED + commandName + " must be between " + std::to_string(*minValue) + " and " + std::to_string(*maxValue) + RESET);
    }

    uint32_t arbitrationId = 0;
    uint32_t valueBits;
    std::array<uint8_t, 8> data = {};

    std::visit([&](auto &&cmd)
               {
                   using T = std::decay_t<decltype(cmd)>;
                   if constexpr (std::is_same_v<T, MotorControl> || std::is_same_v<T, SystemControl>)
                   {
                       arbitrationId = static_cast<uint32_t>(cmd) + deviceId;
                   } },
               command);

    std::memcpy(&valueBits, &value, sizeof(valueBits));
    std::memcpy(data.data(), &valueBits, sizeof(valueBits));

    SendCanFrame(arbitrationId, 8, data);
}

uint64_t SparkBase::ReadPeriodicStatus(Status period) const
{
    constexpr int CACHE_TIMEOUT_MS = 100;
    constexpr int READ_TIMEOUT_US = 20000;

    auto now = std::chrono::steady_clock::now();
    auto it = cachedStatus.find(period);
    if (it != cachedStatus.end())
    {
        auto &[value, timestamp] = it->second;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp).count() < CACHE_TIMEOUT_MS)
        {
            return value;
        }
    }

    struct can_frame response;
    struct timeval tv = {0, READ_TIMEOUT_US};
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(soc, &read_fds);

    int ret = select(soc + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0)
    {
        ssize_t bytesRead = read(soc, &response, sizeof(response));
        if (bytesRead > 0)
        {
            uint32_t receivedArbitrationId = response.can_id & CAN_EFF_MASK;
            uint32_t expectedArbitrationId = static_cast<uint32_t>(period) | deviceId;

            if (receivedArbitrationId == expectedArbitrationId)
            {
                uint64_t newValue = 0;
                for (int i = 0; i < 8; i++)
                {
                    newValue |= static_cast<uint64_t>(response.data[i]) << (8 * i);
                }
                cachedStatus[period] = {newValue, now};
                return newValue;
            }
        }
    }

    if (it != cachedStatus.end())
    {
        return it->second.first;
    }

    return 0;
}

void SparkBase::SetParameter(Parameter parameterId, uint8_t parameterType, std::string parameterName, std::variant<float, uint32_t, uint16_t, uint8_t, bool> value, std::optional<float> minValue, std::optional<float> maxValue, std::optional<std::string> customErrorMessage)
{
    uint32_t arbitrationId = 0x205C000 | (static_cast<uint32_t>(parameterId) << 6) | deviceId;
    std::array<uint8_t, 8> data = {};

    auto throwRangeError = [&](auto min, auto max)
    {
        if (customErrorMessage)
            throw std::out_of_range(RED + *customErrorMessage + RESET);
        else
            throw std::out_of_range(RED + parameterName + " must be between " + std::to_string(min) + " and " + std::to_string(max) + RESET);
    };

    std::visit([&](auto &&v)
               {
                   using T = std::decay_t<decltype(v)>;
                   if constexpr (std::is_floating_point_v<T>)
                   {
                       if (!std::isfinite(v))
                           throw std::invalid_argument(RED + parameterName + " must be a finite number." + RESET);

                       if (minValue && maxValue && (v < *minValue || v > *maxValue))
                           throwRangeError(*minValue, *maxValue);

                       std::memcpy(data.data(), &v, sizeof(v));
                   }
                   else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
                   {
                       if (minValue && maxValue)
                       {
                           auto typedMin = static_cast<T>(*minValue);
                           auto typedMax = static_cast<T>(*maxValue);
                           if (v < typedMin || v > typedMax)
                               throwRangeError(typedMin, typedMax);
                       }
                       std::memcpy(data.data(), &v, sizeof(v));
                   }
                   else if constexpr (std::is_same_v<T, bool>)
                   {
                       data[0] = v ? 1 : 0;
                   } },
               value);

    SendCanFrame(arbitrationId, 8, data);
}

void SparkBase::Heartbeat()
{
    struct can_frame frame = {};
    frame.can_id = 0x2052C80 | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    std::memset(frame.data, 0, sizeof(frame.data));

    uint32_t byteIndex = deviceId >> 3;
    uint32_t bitShift = deviceId & 0x07;

    frame.data[byteIndex] = 1 << bitShift;

    std::array<uint8_t, 8> dataArray;
    std::memcpy(dataArray.data(), frame.data, sizeof(frame.data));

    SendCanFrame(frame.can_id, frame.can_dlc, dataArray);
}

void SparkBase::BurnFlash()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::BurnFlash) + deviceId;

    std::array<uint8_t, 8> data = {0xA3, 0x3A};

    SendCanFrame(arbitrationId, 2, data);
}

void SparkBase::FactoryDefaults()
{
    uint32_t arbitrationId =
        static_cast<uint32_t>(SystemControl::FactoryDefaults) + deviceId;

    std::array<uint8_t, 8> data = {0x01};

    SendCanFrame(arbitrationId, 5, data);
}

void SparkBase::FactoryReset()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::FactoryReset) + deviceId;

    std::array<uint8_t, 8> data = {0x01};

    SendCanFrame(arbitrationId, 5, data);
}

void SparkBase::Identify()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::Identify) + deviceId;

    std::array<uint8_t, 8> data = {};

    SendCanFrame(arbitrationId, 4, data);
}

void SparkBase::ResetFaults()
{
    SendControlMessage(SystemControl::ResetFaults, "ResetFaults", 0.0f);
}

void SparkBase::ClearStickyFaults()
{
    SendControlMessage(SystemControl::ClearStickyFaults, "ClearStickyFaults", 0.0f);
}

// MotorControl //

void SparkBase::SetAppliedOutput(float appliedOutput)
{
    SendControlMessage(MotorControl::AppliedOutput, "Applied Output", appliedOutput, -1.0f, 1.0f);
}

void SparkBase::SetVelocity(float velocity)
{
    SendControlMessage(MotorControl::Velocity, "Velocity", velocity);
}

void SparkBase::SetSmartVelocity(float smartVelocity)
{
    SendControlMessage(MotorControl::SmartVelocity, "Smart Velocity", smartVelocity);
}

void SparkBase::SetPosition(float position)
{
    SendControlMessage(MotorControl::Position, "Position", position);
}

void SparkBase::SetVoltage(float voltage)
{
    SendControlMessage(MotorControl::Voltage, "Voltage", voltage);
}

void SparkBase::SetCurrent(float current)
{
    SendControlMessage(MotorControl::Current, "Current", current);
}

void SparkBase::SetSmartMotion(float smartMotion)
{
    SendControlMessage(MotorControl::SmartMotion, "Smart Motion", smartMotion);
}

// Status //

// Period 0
float SparkBase::GetAppliedOutput() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    int16_t appliedOutput = static_cast<int16_t>(status & 0xFFFF);

    return static_cast<float>(appliedOutput) / 32768.0f;
}

uint16_t SparkBase::GetFaults() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    return (status >> 16) & 0xFFFF;
}

uint16_t SparkBase::GetStickyFaults() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    return (status >> 32) & 0xFFFF;
}

bool SparkBase::GetInverted() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 56) & 0x01;
}

bool SparkBase::GetIdleMode() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 57) & 0x01;
}

bool SparkBase::IsFollower() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 58) & 0x01;
}

// Period 1 //
float SparkBase::GetVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    return *reinterpret_cast<const float *>(&status);
}

float SparkBase::GetTemperature() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    return static_cast<float>((status >> 32) & 0xFF);
}

float SparkBase::GetVoltage() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    uint16_t voltage = (status >> 40) & 0xFFF;

    return static_cast<float>(voltage) / 128.0f;
}

float SparkBase::GetCurrent() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    uint16_t current = (status >> 52) & 0xFFF;

    return static_cast<float>(current) / 32.0f;
}

// Period 2 //
float SparkBase::GetPosition() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period2);

    return *reinterpret_cast<const float *>(&rawData);
}

// Period 3 //
float SparkBase::GetAnalogVoltage() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    uint16_t analogVoltage = status & 0x3FF;

    return static_cast<float>(analogVoltage) / 256.0f;
}

float SparkBase::GetAnalogVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    int32_t analogVelocity = (status >> 10) & 0x3FFFFF;

    if (analogVelocity & 0x200000) // Sign extend if negative
        analogVelocity |= 0xFFC00000;

    return static_cast<float>(analogVelocity) / analogVelocityConversion;
}

float SparkBase::GetAnalogPosition() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    return *reinterpret_cast<const float *>(&status) / analogPositionConversion;
}

// Period 4 //
float SparkBase::GetAlternateEncoderVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period4);
    return *reinterpret_cast<const float *>(&status);
}

float SparkBase::GetAlternateEncoderPosition() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period4);
    return *reinterpret_cast<const float *>((const char *)&status + 4);
}

// Parameters //

// Basic //

void SparkBase::SetMotorType(uint8_t type)
{
    SetParameter(Parameter::kMotorType, PARAM_TYPE_UINT, "Motor Type", type, 0, 1,
                 "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless).");
}

void SparkBase::SetSensorType(uint8_t type)
{
    SetParameter(Parameter::kSensorType, PARAM_TYPE_UINT, "Sensor Type", type, 0, 2,
                 "Invalid sensor type. Must be 0 (No Sensor), 1 (Hall Sensor), or 2 (Encoder).");
}

void SparkBase::SetIdleMode(uint8_t mode)
{
    SetParameter(Parameter::kIdleMode, PARAM_TYPE_UINT, "Idle Mode", mode, 0, 1,
                 "Invalid idle mode. Must be 0 (Coast) or 1 (Brake).");
}

void SparkBase::SetInputDeadband(float deadband)
{
    SetParameter(Parameter::kInputDeadband, PARAM_TYPE_FLOAT, "Input Deadband", deadband);
}

void SparkBase::SetInverted(bool inverted)
{
    SetParameter(Parameter::kInverted, PARAM_TYPE_BOOL, "Inverted", inverted);
}

void SparkBase::SetRampRate(float rate)
{
    SetParameter(Parameter::kRampRate, PARAM_TYPE_FLOAT, "Ramp Rate", rate);
}

// Advanced //

void SparkBase::SetMotorKv(uint16_t kv)
{
    SetParameter(Parameter::kMotorKv, PARAM_TYPE_UINT, "Motor Kv", kv);
}

void SparkBase::SetMotorR(uint16_t r)
{
    SetParameter(Parameter::kMotorR, PARAM_TYPE_UINT, "Motor Resistance", r);
}

void SparkBase::SetMotorL(uint16_t l)
{
    SetParameter(Parameter::kMotorL, PARAM_TYPE_UINT, "Motor Inductance", l);
}

// Closed Loop //

void SparkBase::SetCtrlType(uint8_t type)
{
    SetParameter(Parameter::kCtrlType, PARAM_TYPE_UINT, "Control Type", type, 0, 3,
                 "Invalid control type. Must be 0 (Duty Cycle), 1 (Velocity), 2 (Voltage), or 3 (Position).");
}

void SparkBase::SetFeedbackSensorPID0(uint16_t sensor)
{
    SetParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, "Feedback Sensor PID0", sensor);
}

void SparkBase::SetClosedLoopVoltageMode(uint8_t mode)
{
    SetParameter(Parameter::kClosedLoopVoltageMode, PARAM_TYPE_UINT, "Closed Loop Voltage Mode", mode, 0, 2,
                 "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control Loop Voltage Output Mode) or 2 (Voltage Compensation Mode).");
}

void SparkBase::SetCompensatedNominalVoltage(float voltage)
{
    SetParameter(Parameter::kCompensatedNominalVoltage, PARAM_TYPE_FLOAT, "Compensated Nominal Voltage", voltage);
}

void SparkBase::SetPositionPIDWrapEnable(bool enable)
{
    SetParameter(Parameter::kPositionPIDWrapEnable, PARAM_TYPE_BOOL, "Position PID Wrap Enable", enable);
}

void SparkBase::SetPositionPIDMinInput(float minInput)
{
    SetParameter(Parameter::kPositionPIDMinInput, PARAM_TYPE_FLOAT, "Position PID Min Input", minInput);
}

void SparkBase::SetPositionPIDMaxInput(float maxInput)
{
    SetParameter(Parameter::kPositionPIDMaxInput, PARAM_TYPE_FLOAT, "Position PID Max Input", maxInput);
}

// Brushless //

void SparkBase::SetPolePairs(uint16_t pairs)
{
    SetParameter(Parameter::kPolePairs, PARAM_TYPE_UINT, "Pole Pairs", pairs);
}

// Current Limit //

void SparkBase::SetCurrentChop(float chop)
{
    SetParameter(Parameter::kCurrentChop, PARAM_TYPE_FLOAT, "Current Chop", chop, 0, 125);
}

void SparkBase::SetCurrentChopCycles(uint16_t cycles)
{
    SetParameter(Parameter::kCurrentChopCycles, PARAM_TYPE_UINT, "Current Chop Cycles", cycles);
}

void SparkBase::SetSmartCurrentStallLimit(uint16_t limit)
{
    SetParameter(Parameter::kSmartCurrentStallLimit, PARAM_TYPE_UINT, "Smart Current Stall Limit", limit);
}

void SparkBase::SetSmartCurrentFreeLimit(uint16_t limit)
{
    SetParameter(Parameter::kSmartCurrentFreeLimit, PARAM_TYPE_UINT, "Smart Current Free Limit", limit);
}

void SparkBase::SetSmartCurrentConfig(uint16_t config)
{
    SetParameter(Parameter::kSmartCurrentConfig, PARAM_TYPE_UINT, "Smart Current Config", config);
}

// PIDF //

void SparkBase::SetP(uint8_t slot, float p)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kP_0;
        break;
    case 1:
        param = Parameter::kP_1;
        break;
    case 2:
        param = Parameter::kP_2;
        break;
    case 3:
        param = Parameter::kP_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "P", p);
}

void SparkBase::SetI(uint8_t slot, float i)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kI_0;
        break;
    case 1:
        param = Parameter::kI_1;
        break;
    case 2:
        param = Parameter::kI_2;
        break;
    case 3:
        param = Parameter::kI_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "I", i);
}

void SparkBase::SetD(uint8_t slot, float d)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kD_0;
        break;
    case 1:
        param = Parameter::kD_1;
        break;
    case 2:
        param = Parameter::kD_2;
        break;
    case 3:
        param = Parameter::kD_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "D", d);
}

void SparkBase::SetF(uint8_t slot, float f)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kF_0;
        break;
    case 1:
        param = Parameter::kF_1;
        break;
    case 2:
        param = Parameter::kF_2;
        break;
    case 3:
        param = Parameter::kF_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "F", f);
}

void SparkBase::SetIZone(uint8_t slot, float iZone)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kIZone_0;
        break;
    case 1:
        param = Parameter::kIZone_1;
        break;
    case 2:
        param = Parameter::kIZone_2;
        break;
    case 3:
        param = Parameter::kIZone_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "IZone", iZone);
}

void SparkBase::SetDFilter(uint8_t slot, float dFilter)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kDFilter_0;
        break;
    case 1:
        param = Parameter::kDFilter_1;
        break;
    case 2:
        param = Parameter::kDFilter_2;
        break;
    case 3:
        param = Parameter::kDFilter_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "DFilter", dFilter);
}

void SparkBase::SetOutputMin(uint8_t slot, float min)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kOutputMin_0;
        break;
    case 1:
        param = Parameter::kOutputMin_1;
        break;
    case 2:
        param = Parameter::kOutputMin_2;
        break;
    case 3:
        param = Parameter::kOutputMin_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Output Min", min);
}

void SparkBase::SetOutputMax(uint8_t slot, float max)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kOutputMax_0;
        break;
    case 1:
        param = Parameter::kOutputMax_1;
        break;
    case 2:
        param = Parameter::kOutputMax_2;
        break;
    case 3:
        param = Parameter::kOutputMax_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Output Max", max);
}

// Limits //

void SparkBase::SetHardLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kHardLimitFwdEn, PARAM_TYPE_BOOL, "Hard Limit Forward Enable", enable);
}

void SparkBase::SetHardLimitRevEn(bool enable)
{
    SetParameter(Parameter::kHardLimitRevEn, PARAM_TYPE_BOOL, "Hard Limit Reverse Enable", enable);
}

void SparkBase::SetLimitSwitchFwdPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, "Limit Switch Forward Polarity", polarity);
}

void SparkBase::SetLimitSwitchRevPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, "Limit Switch Reverse Polarity", polarity);
}

void SparkBase::SetSoftLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitFwdEn, PARAM_TYPE_BOOL, "Soft Limit Forward Enable", enable);
}

void SparkBase::SetSoftLimitRevEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitRevEn, PARAM_TYPE_BOOL, "Soft Limit Reverse Enable", enable);
}

void SparkBase::SetSoftLimitFwd(float limit)
{
    SetParameter(Parameter::kSoftLimitFwd, PARAM_TYPE_FLOAT, "Soft Limit Forward", limit);
}

void SparkBase::SetSoftLimitRev(float limit)
{
    SetParameter(Parameter::kSoftLimitRev, PARAM_TYPE_FLOAT, "Soft Limit Reverse", limit);
}

// Follower //

void SparkBase::SetFollowerID(uint32_t id)
{
    SetParameter(Parameter::kFollowerID, PARAM_TYPE_UINT, "Follower ID", id);
}

void SparkBase::SetFollowerConfig(uint32_t config)
{
    SetParameter(Parameter::kFollowerConfig, PARAM_TYPE_UINT, "Follower Config", config);
}

// Encoder Port //

void SparkBase::SetEncoderCountsPerRev(uint16_t counts)
{
    SetParameter(Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, "Encoder Counts Per Revolution", counts);
}

void SparkBase::SetEncoderAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, "Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetEncoderSampleDelta(uint8_t delta)
{
    SetParameter(Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, "Encoder Sample Delta", delta, 1, 255);
}

void SparkBase::SetEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, "Encoder Inverted", inverted);
}

void SparkBase::SetPositionConversionFactor(float factor)
{
    SetParameter(Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, "Position Conversion Factor", factor);
}

void SparkBase::SetVelocityConversionFactor(float factor)
{
    SetParameter(Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, "Velocity Conversion Factor", factor);
}

void SparkBase::SetClosedLoopRampRate(float rampRate)
{
    SetParameter(Parameter::kClosedLoopRampRate, PARAM_TYPE_FLOAT, "Closed Loop Ramp Rate", rampRate);
}

void SparkBase::SetHallSensorSampleRate(float rate)
{
    SetParameter(Parameter::kHallSensorSampleRate, PARAM_TYPE_FLOAT, "Hall Sensor Sample Rate", rate);
}

void SparkBase::SetHallSensorAverageDepth(uint16_t depth)
{
    SetParameter(Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, "Hall Sensor Average Depth", depth);
}

// Smart Motion //

void SparkBase::SetSmartMotionMaxVelocity(uint8_t slot, float maxVel)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSmartMotionMaxVelocity_0;
        break;
    case 1:
        param = Parameter::kSmartMotionMaxVelocity_1;
        break;
    case 2:
        param = Parameter::kSmartMotionMaxVelocity_2;
        break;
    case 3:
        param = Parameter::kSmartMotionMaxVelocity_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Smart Motion Max Velocity", maxVel);
}

void SparkBase::SetSmartMotionMaxAccel(uint8_t slot, float maxAccel)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSmartMotionMaxAccel_0;
        break;
    case 1:
        param = Parameter::kSmartMotionMaxAccel_1;
        break;
    case 2:
        param = Parameter::kSmartMotionMaxAccel_2;
        break;
    case 3:
        param = Parameter::kSmartMotionMaxAccel_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Smart Motion Max Accel", maxAccel);
}

void SparkBase::SetSmartMotionMinVelOutput(uint8_t slot, float minVel)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSmartMotionMinVelOutput_0;
        break;
    case 1:
        param = Parameter::kSmartMotionMinVelOutput_1;
        break;
    case 2:
        param = Parameter::kSmartMotionMinVelOutput_2;
        break;
    case 3:
        param = Parameter::kSmartMotionMinVelOutput_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Smart Motion Min Vel Output", minVel);
}

void SparkBase::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error)
{
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSmartMotionAllowedClosedLoopError_0;
        break;
    case 1:
        param = Parameter::kSmartMotionAllowedClosedLoopError_1;
        break;
    case 2:
        param = Parameter::kSmartMotionAllowedClosedLoopError_2;
        break;
    case 3:
        param = Parameter::kSmartMotionAllowedClosedLoopError_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, "Smart Motion Allowed Close Loop Error", error);
}

void SparkBase::SetSmartMotionAccelStrategy(uint8_t slot, float strategy)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionAccelStrategy_0,
        Parameter::kSmartMotionAccelStrategy_1,
        Parameter::kSmartMotionAccelStrategy_2,
        Parameter::kSmartMotionAccelStrategy_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Accel Strategy", strategy);
}

void SparkBase::SetIMaxAccum(uint8_t slot, float maxAccum)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kIMaxAccum_0,
        Parameter::kIMaxAccum_1,
        Parameter::kIMaxAccum_2,
        Parameter::kIMaxAccum_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "IMaxAccum", maxAccum);
}

void SparkBase::SetSlot3Placeholder1(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder1_0,
        Parameter::kSlot3Placeholder1_1,
        Parameter::kSlot3Placeholder1_2,
        Parameter::kSlot3Placeholder1_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 1", value);
}

void SparkBase::SetSlot3Placeholder2(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder2_0,
        Parameter::kSlot3Placeholder2_1,
        Parameter::kSlot3Placeholder2_2,
        Parameter::kSlot3Placeholder2_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 2", value);
}

void SparkBase::SetSlot3Placeholder3(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder3_0,
        Parameter::kSlot3Placeholder3_1,
        Parameter::kSlot3Placeholder3_2,
        Parameter::kSlot3Placeholder3_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 3", value);
}

// Analog Sensor //

void SparkBase::SetAnalogPositionConversion(float factor)
{
    SetParameter(Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, "Analog Position Conversion", factor);
    analogPositionConversion = factor;
}

void SparkBase::SetAnalogVelocityConversion(float factor)
{
    SetParameter(Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, "Analog Velocity Conversion", factor);
    analogVelocityConversion = factor;
}

void SparkBase::SetAnalogAverageDepth(uint16_t depth)
{
    SetParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, "Analog Average Depth", depth);
}

void SparkBase::SetAnalogSensorMode(uint8_t mode)
{
    SetParameter(Parameter::kAnalogSensorMode, PARAM_TYPE_UINT, "Analog Sensor Mode", mode, 0, 1,
                 "Invalid analog sensor mode. Must be 0 (Absolute) or 1 (Relative).");
}

void SparkBase::SetAnalogInverted(bool inverted)
{
    SetParameter(Parameter::kAnalogInverted, PARAM_TYPE_BOOL, "Analog Inverted", inverted);
}

void SparkBase::SetAnalogSampleDelta(uint16_t delta)
{
    SetParameter(Parameter::kAnalogSampleDelta, PARAM_TYPE_UINT, "Analog Sample Delta", delta);
}

// Alternate Encoder //

void SparkBase::SetDataPortConfig(uint8_t config)
{
    SetParameter(Parameter::kDataPortConfig, PARAM_TYPE_UINT, "Data Port Config", config, 0, 1,
                 "Invalid data port config. Must be 0 (Default) or 1 (Alternate Encoder Mode).");
}

void SparkBase::SetAltEncoderCountsPerRev(uint16_t counts)
{
    SetParameter(Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, "Alternate Encoder Counts Per Revolution", counts);
}

void SparkBase::SetAltEncoderAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT, "Alternate Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetAltEncoderSampleDelta(uint8_t delta)
{
    SetParameter(Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, "Alternate Encoder Sample Delta", delta, 1, 255);
}

void SparkBase::SetAltEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, "Alternate Encoder Inverted", inverted);
}

void SparkBase::SetAltEncoderPositionFactor(float factor)
{
    SetParameter(Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Position Factor", factor);
}

void SparkBase::SetAltEncoderVelocityFactor(float factor)
{
    SetParameter(Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Velocity Factor", factor);
}

// Duty Cycle Absolute Encoder //

void SparkBase::SetDutyCyclePositionFactor(float factor)
{
    SetParameter(Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, "Duty Cycle Position Factor", factor);
}

void SparkBase::SetDutyCycleVelocityFactor(float factor)
{
    SetParameter(Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, "Duty Cycle Velocity Factor", factor);
}

void SparkBase::SetDutyCycleInverted(bool inverted)
{
    SetParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, "Duty Cycle Inverted", inverted);
}

void SparkBase::SetDutyCycleAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kDutyCycleAverageDepth, PARAM_TYPE_UINT, "Duty Cycle Average Depth", depth, 0, 7,
                 "Invalid average depth. Must be 0 (1 bit), 2 (2 bits), 3 (4 bits), 4 (8 bits), 5 (16 bits), 6 (32 bits), or 7 (64 bits).");
}

void SparkBase::SetDutyCyclePrescalar(uint8_t prescalar)
{
    SetParameter(Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, "Duty Cycle Prescalar", prescalar, 0, 71);
}

void SparkBase::SetDutyCycleZeroOffset(float offset)
{
    SetParameter(Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, "Duty Cycle Zero Offset", offset, 0.0f, 1.0f);
}