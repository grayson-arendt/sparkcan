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
        throw std::out_of_range(
            RED "Invalid CAN bus ID. Must be between 0 and 62." RESET);
    }

    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (soc < 0)
    {
        throw std::system_error(errno, std::generic_category(),
                                std::string(RED) + "Socket creation failed: " + strerror(errno) + std::string(RESET));
    }

    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interfaceName.c_str(), sizeof(ifr.ifr_name) - 1);

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        close(soc);
        throw std::runtime_error(std::string(RED) + "IOCTL failed: " + std::string(strerror(errno)) + std::string(RESET));
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(soc);
        throw std::runtime_error(std::string(RED) + "Binding to interface failed: " +
                                 std::string(strerror(errno)) + std::string(RESET));
    }
}

SparkBase::~SparkBase() { close(soc); }

void SparkBase::SendCanFrame(uint32_t deviceId, uint8_t dlc,
                             const std::array<uint8_t, 8> &data) const
{
    struct can_frame frame = {};

    frame.can_id = deviceId | CAN_EFF_FLAG;
    frame.can_dlc = dlc;

    std::memcpy(frame.data, data.data(), dlc);

    const std::chrono::milliseconds WAIT_TIME(1);
    const int MAX_ATTEMPTS = 1000;

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
            throw std::runtime_error(std::string(RED) + "Error sending CAN frame: " +
                                     std::string(strerror(errno)) + std::string(RESET));
        }
    }

    throw std::runtime_error(
        RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::SendControlMessage(
    std::variant<MotorControl, SystemControl> command, float value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED "Control value must be a finite number." RESET);
    }

    uint32_t arbitrationId = 0;
    uint32_t valueBits;
    std::array<uint8_t, 8> data = {};

    std::visit(
        [&](auto &&cmd)
        {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, MotorControl> ||
                          std::is_same_v<T, SystemControl>)
            {
                arbitrationId = static_cast<uint32_t>(cmd) + deviceId;
            }
        },
        command);

    std::memcpy(&valueBits, &value, sizeof(valueBits));
    std::memcpy(data.data(), &valueBits, sizeof(valueBits));

    SendCanFrame(arbitrationId, 8, data);
}

uint64_t SparkBase::ReadPeriodicStatus(Status period) const
{
    const int CACHE_TIMEOUT_MS = 100;
    const int READ_TIMEOUT_US = 20000;

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

void SparkBase::SetParameter(
    Parameter parameterId, uint8_t parameterType,
    std::variant<float, uint32_t, uint16_t, uint8_t, bool> value)
{
    uint32_t arbitrationId = 0x205C000 | (static_cast<uint32_t>(parameterId) << 6) | deviceId;

    std::array<uint8_t, 8> data = {};

    std::visit(
        [&](auto &&v)
        {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, float>)
            {
                if (!std::isfinite(v))
                {
                    throw std::invalid_argument(RED "Parameter value must be a finite number." RESET);
                }
                std::memcpy(data.data(), &v, sizeof(v));
            }
            else if constexpr (std::is_same_v<T, uint32_t> || std::is_same_v<T, uint16_t>)
            {
                if (v > std::numeric_limits<T>::max())
                {
                    throw std::out_of_range(RED "Parameter value is out of valid range." RESET);
                }
                std::memcpy(data.data(), &v, sizeof(v));
            }
            else if constexpr (std::is_same_v<T, uint8_t>)
            {
                data[0] = v;
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                data[0] = v ? 1 : 0;
            }
            else
            {
                throw std::invalid_argument(RED "Invalid parameter type." RESET);
            }
        },
        value);

    data[4] = parameterType;

    SendCanFrame(arbitrationId, 5, data);
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
    SendControlMessage(SystemControl::ResetFaults, 0.0f);
}

void SparkBase::ClearStickyFaults()
{
    SendControlMessage(SystemControl::ClearStickyFaults, 0.0f);
}

// MotorControl //

void SparkBase::SetAppliedOutput(float appliedOutput)
{
    if (appliedOutput < -1.0f || appliedOutput > 1.0f || !std::isfinite(appliedOutput))
    {
        throw std::out_of_range(RED "Applied output must be a finite number between -1.0 and 1.0." RESET);
    }

    SendControlMessage(MotorControl::AppliedOutput, appliedOutput);
}

void SparkBase::SetVelocity(float velocity)
{
    if (!std::isfinite(velocity))
    {
        throw std::invalid_argument(RED "Velocity must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::Velocity, velocity);
}

void SparkBase::SetSmartVelocity(float smartVelocity)
{
    if (!std::isfinite(smartVelocity))
    {
        throw std::invalid_argument(RED "Smart velocity must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::SmartVelocity, smartVelocity);
}

void SparkBase::SetPosition(float position)
{
    if (!std::isfinite(position))
    {
        throw std::invalid_argument(RED "Position must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::Position, position);
}

void SparkBase::SetVoltage(float voltage)
{
    if (!std::isfinite(voltage))
    {
        throw std::invalid_argument(RED "Voltage must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::Voltage, voltage);
}

void SparkBase::SetCurrent(float current)
{
    if (!std::isfinite(current))
    {
        throw std::invalid_argument(RED "Current must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::Current, current);
}

void SparkBase::SetSmartMotion(float smartMotion)
{
    if (!std::isfinite(smartMotion))
    {
        throw std::invalid_argument(RED "Smart motion must be a finite number." RESET);
    }
    SendControlMessage(MotorControl::SmartMotion, smartMotion);
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

    return static_cast<float>(analogVelocity) / 128.0f;
}

float SparkBase::GetAnalogPosition() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    return *reinterpret_cast<const float *>(&status);
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
    if (type > 1)
    {
        throw std::invalid_argument(
            RED "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless)." RESET);
    }

    SetParameter(Parameter::kMotorType, PARAM_TYPE_UINT, type);
}

void SparkBase::SetSensorType(uint8_t type)
{
    if (type > 2)
    {
        throw std::out_of_range(
            RED "Invalid sensor type. Must be 0 (No Sensor), 1 "
                "(Hall Sensor), or 2 (Encoder)." RESET);
    }

    SetParameter(Parameter::kSensorType, PARAM_TYPE_UINT, type);
}

void SparkBase::SetIdleMode(uint8_t mode)
{
    if (mode > 1)
    {
        throw std::out_of_range(
            RED "Invalid idle mode. Must be 0 (Coast) or 1 (Brake)." RESET);
    }

    SetParameter(Parameter::kIdleMode, PARAM_TYPE_UINT, mode);
}

void SparkBase::SetInputDeadband(float deadband)
{
    if (!std::isfinite(deadband))
    {
        throw std::invalid_argument(RED "Input deadband must be a finite number." RESET);
    }
    SetParameter(Parameter::kInputDeadband, PARAM_TYPE_FLOAT, deadband);
}

void SparkBase::SetInverted(bool inverted)
{
    SetParameter(Parameter::kInverted, PARAM_TYPE_BOOL, inverted);
}

void SparkBase::SetRampRate(float rate)
{
    if (!std::isfinite(rate))
    {
        throw std::invalid_argument(RED "Ramp rate must be a finite number." RESET);
    }
    SetParameter(Parameter::kRampRate, PARAM_TYPE_FLOAT, rate);
}

// Advanced //

void SparkBase::SetMotorKv(uint16_t kv)
{
    if (kv > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Motor Kv value is out of range." RESET);
    }
    SetParameter(Parameter::kMotorKv, PARAM_TYPE_UINT, kv);
}

void SparkBase::SetMotorR(uint16_t r)
{
    if (r > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Motor resistance value is out of range." RESET);
    }
    SetParameter(Parameter::kMotorR, PARAM_TYPE_UINT, r);
}

void SparkBase::SetMotorL(uint16_t l)
{
    if (l > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Motor inductance value is out of range." RESET);
    }
    SetParameter(Parameter::kMotorL, PARAM_TYPE_UINT, l);
}

// Closed Loop //

void SparkBase::SetCtrlType(uint8_t type)
{
    if (type > 3)
    {
        throw std::out_of_range(
            RED "Invalid control type. Must be 0 (Duty Cycle), "
                "1 (Velocity), 2 (Voltage), or 3 (Position)." RESET);
    }

    SetParameter(Parameter::kCtrlType, PARAM_TYPE_UINT, type);
}

void SparkBase::SetFeedbackSensorPID0(uint16_t sensor)
{
    if (sensor > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Feedback sensor PID0 value is out of range." RESET);
    }
    SetParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, sensor);
}

void SparkBase::SetClosedLoopVoltageMode(uint8_t mode)
{
    if (mode > 2)
    {
        throw std::out_of_range(
            RED "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control "
                "Loop Voltage "
                "Output Mode) or 2 (Voltage Compensation Mode)." RESET);
    }

    SetParameter(Parameter::kClosedLoopVoltageMode, PARAM_TYPE_UINT, mode);
}

void SparkBase::SetCompensatedNominalVoltage(float voltage)
{
    if (!std::isfinite(voltage))
    {
        throw std::invalid_argument(RED "Compensated nominal voltage must be a finite number." RESET);
    }
    SetParameter(Parameter::kCompensatedNominalVoltage, PARAM_TYPE_FLOAT, voltage);
}

void SparkBase::SetPositionPIDWrapEnable(bool enable)
{
    SetParameter(Parameter::kPositionPIDWrapEnable, PARAM_TYPE_BOOL, enable);
}

void SparkBase::SetPositionPIDMinInput(float minInput)
{
    if (!std::isfinite(minInput))
    {
        throw std::invalid_argument(RED "Position PID min input must be a finite number." RESET);
    }
    SetParameter(Parameter::kPositionPIDMinInput, PARAM_TYPE_FLOAT, minInput);
}

void SparkBase::SetPositionPIDMaxInput(float maxInput)
{
    if (!std::isfinite(maxInput))
    {
        throw std::invalid_argument(RED "Position PID max input must be a finite number." RESET);
    }
    SetParameter(Parameter::kPositionPIDMaxInput, PARAM_TYPE_FLOAT, maxInput);
}

// Brushless //

void SparkBase::SetPolePairs(uint16_t pairs)
{
    if (pairs > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Pole pairs value is out of range." RESET);
    }
    SetParameter(Parameter::kPolePairs, PARAM_TYPE_UINT, pairs);
}

// Current Limit //

void SparkBase::SetCurrentChop(float chop)
{
    if (!std::isfinite(chop) || chop > 125.0f)
    {
        throw std::out_of_range(RED "Invalid current chop. Max value is 125." RESET);
    }

    SetParameter(Parameter::kCurrentChop, PARAM_TYPE_FLOAT, chop);
}

void SparkBase::SetCurrentChopCycles(uint16_t cycles)
{
    if (cycles > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Current chop cycles value is out of range." RESET);
    }
    SetParameter(Parameter::kCurrentChopCycles, PARAM_TYPE_UINT, cycles);
}

void SparkBase::SetSmartCurrentStallLimit(uint16_t limit)
{
    if (limit > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Smart current stall limit value is out of range." RESET);
    }
    SetParameter(Parameter::kSmartCurrentStallLimit, PARAM_TYPE_UINT, limit);
}

void SparkBase::SetSmartCurrentFreeLimit(uint16_t limit)
{
    if (limit > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Smart current free limit value is out of range." RESET);
    }
    SetParameter(Parameter::kSmartCurrentFreeLimit, PARAM_TYPE_UINT, limit);
}

void SparkBase::SetSmartCurrentConfig(uint16_t config)
{
    if (config > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Smart current config value is out of range." RESET);
    }
    SetParameter(Parameter::kSmartCurrentConfig, PARAM_TYPE_UINT, config);
}

// PIDF //

void SparkBase::SetP(uint8_t slot, float p)
{
    if (!std::isfinite(p))
    {
        throw std::invalid_argument(RED "P value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, p);
}

void SparkBase::SetI(uint8_t slot, float i)
{
    if (!std::isfinite(i))
    {
        throw std::invalid_argument(RED "I value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, i);
}

void SparkBase::SetD(uint8_t slot, float d)
{
    if (!std::isfinite(d))
    {
        throw std::invalid_argument(RED "D value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, d);
}

void SparkBase::SetF(uint8_t slot, float f)
{
    if (!std::isfinite(f))
    {
        throw std::invalid_argument(RED "F value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, f);
}

void SparkBase::SetIZone(uint8_t slot, float iZone)
{
    if (!std::isfinite(iZone))
    {
        throw std::invalid_argument(RED "IZone value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, iZone);
}

void SparkBase::SetDFilter(uint8_t slot, float dFilter)
{
    if (!std::isfinite(dFilter))
    {
        throw std::invalid_argument(RED "DFilter value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, dFilter);
}

void SparkBase::SetOutputMin(uint8_t slot, float min)
{
    if (!std::isfinite(min))
    {
        throw std::invalid_argument(RED "Output min value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, min);
}

void SparkBase::SetOutputMax(uint8_t slot, float max)
{
    if (!std::isfinite(max))
    {
        throw std::invalid_argument(RED "Output max value must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, max);
}

// Limits //

void SparkBase::SetHardLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kHardLimitFwdEn, PARAM_TYPE_BOOL, enable);
}

void SparkBase::SetHardLimitRevEn(bool enable)
{
    SetParameter(Parameter::kHardLimitRevEn, PARAM_TYPE_BOOL, enable);
}

void SparkBase::SetLimitSwitchFwdPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, polarity);
}

void SparkBase::SetLimitSwitchRevPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, polarity);
}

void SparkBase::SetSoftLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitFwdEn, PARAM_TYPE_BOOL, enable);
}

void SparkBase::SetSoftLimitRevEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitRevEn, PARAM_TYPE_BOOL, enable);
}

void SparkBase::SetSoftLimitFwd(float limit)
{
    if (!std::isfinite(limit))
    {
        throw std::invalid_argument(RED "Soft limit forward value must be a finite number." RESET);
    }
    SetParameter(Parameter::kSoftLimitFwd, PARAM_TYPE_FLOAT, limit);
}

void SparkBase::SetSoftLimitRev(float limit)
{
    if (!std::isfinite(limit))
    {
        throw std::invalid_argument(RED "Soft limit reverse value must be a finite number." RESET);
    }
    SetParameter(Parameter::kSoftLimitRev, PARAM_TYPE_FLOAT, limit);
}

// Follower //

void SparkBase::SetFollowerID(uint32_t id)
{
    if (id > std::numeric_limits<uint32_t>::max())
    {
        throw std::out_of_range(RED "Follower ID value is out of range." RESET);
    }
    SetParameter(Parameter::kFollowerID, PARAM_TYPE_UINT, id);
}

void SparkBase::SetFollowerConfig(uint32_t config)
{
    if (config > std::numeric_limits<uint32_t>::max())
    {
        throw std::out_of_range(RED "Follower config value is out of range." RESET);
    }
    SetParameter(Parameter::kFollowerConfig, PARAM_TYPE_UINT, config);
}

// Encoder Port //

void SparkBase::SetEncoderCountsPerRev(uint16_t counts)
{
    if (counts > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Encoder counts per revolution value is out of range." RESET);
    }
    SetParameter(Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, counts);
}

void SparkBase::SetEncoderAverageDepth(uint8_t depth)
{
    if (depth > 64 || depth == 0)
    {
        throw std::out_of_range(
            RED "Invalid average depth. Must be between 1 or 64." RESET);
    }

    SetParameter(Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, depth);
}

void SparkBase::SetEncoderSampleDelta(uint8_t delta)
{
    if (delta == 0)
    {
        throw std::out_of_range(
            RED "Invalid sample delta. Must be between 1 or 255." RESET);
    }

    SetParameter(Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, delta);
}

void SparkBase::SetEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, inverted);
}

void SparkBase::SetPositionConversionFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Position conversion factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetVelocityConversionFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Velocity conversion factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetClosedLoopRampRate(float rampRate)
{
    if (!std::isfinite(rampRate))
    {
        throw std::invalid_argument(RED "Closed loop ramp rate must be a finite number." RESET);
    }
    SetParameter(Parameter::kClosedLoopRampRate, PARAM_TYPE_FLOAT, rampRate);
}

void SparkBase::SetHallSensorSampleRate(float rate)
{
    if (!std::isfinite(rate))
    {
        throw std::invalid_argument(RED "Hall sensor sample rate must be a finite number." RESET);
    }
    SetParameter(Parameter::kHallSensorSampleRate, PARAM_TYPE_FLOAT, rate);
}

void SparkBase::SetHallSensorAverageDepth(uint16_t depth)
{
    if (depth > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Hall sensor average depth value is out of range." RESET);
    }
    SetParameter(Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, depth);
}

// Smart Motion //

void SparkBase::SetSmartMotionMaxVelocity(uint8_t slot, float velocity)
{
    if (!std::isfinite(velocity))
    {
        throw std::invalid_argument(RED "Smart motion max velocity must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, velocity);
}

void SparkBase::SetSmartMotionMaxAccel(uint8_t slot, float accel)
{
    if (!std::isfinite(accel))
    {
        throw std::invalid_argument(RED "Smart motion max acceleration must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, accel);
}

void SparkBase::SetSmartMotionMinVelOutput(uint8_t slot, float minVel)
{
    if (!std::isfinite(minVel))
    {
        throw std::invalid_argument(RED "Smart motion min velocity output must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, minVel);
}

void SparkBase::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error)
{
    if (!std::isfinite(error))
    {
        throw std::invalid_argument(RED "Smart motion allowed closed loop error must be a finite number." RESET);
    }
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

    SetParameter(param, PARAM_TYPE_FLOAT, error);
}

void SparkBase::SetSmartMotionAccelStrategy(uint8_t slot, float strategy)
{
    if (!std::isfinite(strategy))
    {
        throw std::invalid_argument(RED "Smart motion acceleration strategy must be a finite number." RESET);
    }
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSmartMotionAccelStrategy_0;
        break;
    case 1:
        param = Parameter::kSmartMotionAccelStrategy_1;
        break;
    case 2:
        param = Parameter::kSmartMotionAccelStrategy_2;
        break;
    case 3:
        param = Parameter::kSmartMotionAccelStrategy_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, strategy);
}

void SparkBase::SetIMaxAccum(uint8_t slot, float maxAccum)
{
    if (!std::isfinite(maxAccum))
    {
        throw std::invalid_argument(RED "IMaxAccum value must be a finite number." RESET);
    }
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kIMaxAccum_0;
        break;
    case 1:
        param = Parameter::kIMaxAccum_1;
        break;
    case 2:
        param = Parameter::kIMaxAccum_2;
        break;
    case 3:
        param = Parameter::kIMaxAccum_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, maxAccum);
}

void SparkBase::SetSlot3Placeholder1(uint8_t slot, float value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED "Slot 3 placeholder 1 value must be a finite number." RESET);
    }
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSlot3Placeholder1_0;
        break;
    case 1:
        param = Parameter::kSlot3Placeholder1_1;
        break;
    case 2:
        param = Parameter::kSlot3Placeholder1_2;
        break;
    case 3:
        param = Parameter::kSlot3Placeholder1_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, value);
}

void SparkBase::SetSlot3Placeholder2(uint8_t slot, float value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED "Slot 3 placeholder 2 value must be a finite number." RESET);
    }
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSlot3Placeholder2_0;
        break;
    case 1:
        param = Parameter::kSlot3Placeholder2_1;
        break;
    case 2:
        param = Parameter::kSlot3Placeholder2_2;
        break;
    case 3:
        param = Parameter::kSlot3Placeholder2_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, value);
}

void SparkBase::SetSlot3Placeholder3(uint8_t slot, float value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED "Slot 3 placeholder 3 value must be a finite number." RESET);
    }
    Parameter param;
    switch (slot)
    {
    case 0:
        param = Parameter::kSlot3Placeholder3_0;
        break;
    case 1:
        param = Parameter::kSlot3Placeholder3_1;
        break;
    case 2:
        param = Parameter::kSlot3Placeholder3_2;
        break;
    case 3:
        param = Parameter::kSlot3Placeholder3_3;
        break;
    default:
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(param, PARAM_TYPE_FLOAT, value);
}

// Analog Sensor //

void SparkBase::SetAnalogPositionConversion(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Analog position conversion factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetAnalogVelocityConversion(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Analog velocity conversion factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetAnalogAverageDepth(uint16_t depth)
{
    if (depth > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Analog average depth value is out of range." RESET);
    }
    SetParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, depth);
}

void SparkBase::SetAnalogSensorMode(uint8_t mode)
{
    if (mode > 1)
    {
        throw std::out_of_range(
            RED "Invalid analog sensor mode. Must be 0 (Absolute) or 1 (Relative)." RESET);
    }

    SetParameter(Parameter::kAnalogSensorMode, PARAM_TYPE_UINT, mode);
}

void SparkBase::SetAnalogInverted(bool inverted)
{
    SetParameter(Parameter::kAnalogInverted, PARAM_TYPE_BOOL, inverted);
}

void SparkBase::SetAnalogSampleDelta(uint16_t delta)
{
    if (delta > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Analog sample delta value is out of range." RESET);
    }
    SetParameter(Parameter::kAnalogSampleDelta, PARAM_TYPE_UINT, delta);
}

// Alternate Encoder //

void SparkBase::SetDataPortConfig(uint8_t config)
{
    if (config > 1)
    {
        throw std::out_of_range(
            RED "Invalid data port config. Must be 0 (Default) "
                "or 1 (Alternate Encoder Mode)." RESET);
    }

    SetParameter(Parameter::kDataPortConfig, PARAM_TYPE_UINT, config);
}

void SparkBase::SetAltEncoderCountsPerRev(uint16_t counts)
{
    if (counts > std::numeric_limits<uint16_t>::max())
    {
        throw std::out_of_range(RED "Alternate encoder counts per revolution value is out of range." RESET);
    }
    SetParameter(Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, counts);
}

void SparkBase::SetAltEncoderAverageDepth(uint8_t depth)
{
    if (depth > 64 || depth == 0)
    {
        throw std::out_of_range(
            RED "Invalid average depth. Must be between 1 or 64." RESET);
    }

    SetParameter(Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT, depth);
}

void SparkBase::SetAltEncoderSampleDelta(uint8_t delta)
{
    if (delta == 0)
    {
        throw std::out_of_range(
            RED "Invalid sample delta. Must be between 1 or 255." RESET);
    }

    SetParameter(Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, delta);
}

void SparkBase::SetAltEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, inverted);
}

void SparkBase::SetAltEncoderPositionFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Alternate encoder position factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetAltEncoderVelocityFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Alternate encoder velocity factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT, factor);
}

// Duty Cycle Absolute Encoder //

void SparkBase::SetDutyCyclePositionFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Duty cycle position factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetDutyCycleVelocityFactor(float factor)
{
    if (!std::isfinite(factor))
    {
        throw std::invalid_argument(RED "Duty cycle velocity factor must be a finite number." RESET);
    }
    SetParameter(Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, factor);
}

void SparkBase::SetDutyCycleInverted(bool inverted)
{
    SetParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, inverted);
}

void SparkBase::SetDutyCycleAverageDepth(uint8_t depth)
{
    if (depth > 7)
    {
        throw std::out_of_range(
            RED "Invalid average depth. Must be 0 (1 bit), 2 "
                "(2 bits), 3 (4 bits), 4 (8 bits), 5 "
                "(16 bits), 6 (32 bits), or 7 (64 bits)." RESET);
    }

    SetParameter(Parameter::kDutyCycleAverageDepth, PARAM_TYPE_UINT, depth);
}

void SparkBase::SetDutyCyclePrescalar(uint8_t prescalar)
{
    if (prescalar > 71)
    {
        throw std::out_of_range(RED "Invalid prescalar. Max value is 71." RESET);
    }

    SetParameter(Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, prescalar);
}

void SparkBase::SetDutyCycleZeroOffset(float offset)
{
    if (!std::isfinite(offset) || offset > 1.0f || offset < 0.0f)
    {
        throw std::out_of_range(RED "Invalid offset. Must be between 0 and 1." RESET);
    }

    SetParameter(Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, offset);
}
