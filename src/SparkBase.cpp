/**
 * @file SparkBase.cpp
 * @brief Source file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#include "SparkBase.hpp"

// ANSI colors
#define RED "\033[31m"
#define YELLOW "\033[33m"
#define CYAN "\033[36m"
#define GREEN "\033[32m"
#define RESET "\033[0m"

// CAN message constants
#define DEVICE_TYPE 0x02
#define MANUFACTURER 0x05
#define READ_TIMEOUT_US 20000

// Parameter types
static constexpr uint8_t PARAM_TYPE_UINT  = 0x01;
static constexpr uint8_t PARAM_TYPE_FLOAT = 0x02;
static constexpr uint8_t PARAM_TYPE_BOOL  = 0x03;

// Periodic status frame scale factors
static constexpr float P0_DUTY_CYCLE_SCALE = 1 << 15;
static constexpr float P1_VOLTAGE_SCALE    = 1 << 7;
static constexpr float P1_CURRENT_SCALE    = 1 << 5;
static constexpr float P2_IACCUM_SCALE     = 1000.0f;
static constexpr float P3_VOLTAGE_SCALE    = 1 << 8;
static constexpr float P3_VELOCITY_SCALE   = 1 << 15;

template <typename T>
static T unpack(const uint8_t* b, int offset = 0)
{
  T v;
  std::memcpy(&v, b + offset, sizeof(T));
  return v;
}

SparkBase::SparkBase(const std::string& interfaceName, uint8_t deviceId) : deviceId_(deviceId)
{
  if (deviceId_ > 62)
  {
    throw std::out_of_range(RED "Invalid CAN bus ID. Must be between 0 and 62." RESET);
  }

  std::strncpy(interfaceName_, interfaceName.c_str(), IFNAMSIZ - 1);
  interfaceName_[IFNAMSIZ - 1] = '\0';

  // Create CAN socket
  SparkBase::soc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (SparkBase::soc_ < 0)
  {
    char errBuf[256];
    std::snprintf(errBuf,
                  sizeof(errBuf),
                  RED
                  "Socket creation failed: %s\nPossible causes:\n1. CAN modules not loaded\n2. System resource "
                  "limitations" RESET,
                  strerror(errno));
    throw std::system_error(errno, std::generic_category(), errBuf);
  }

  // Prepare CAN interface request structure
  std::memset(&ifr_, 0, sizeof(ifr_));
  std::strncpy(ifr_.ifr_name, interfaceName_, sizeof(ifr_.ifr_name) - 1);

  // Get CAN interface index
  if (ioctl(soc_, SIOCGIFINDEX, &ifr_) < 0)
  {
    close(soc_);
    char errBuf[256];
    std::snprintf(errBuf,
                  sizeof(errBuf),
                  RED
                  "IOCTL failed: %s\nPossible causes:\n1. CAN interface does not exist\n2. CAN bus not initialized\n3. "
                  "CAN interface is not up" RESET,
                  strerror(errno));
    throw std::runtime_error(errBuf);
  }

  // Set up CAN address and bind socket to interface
  addr_.can_family  = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  if (bind(soc_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
  {
    close(soc_);
    throw std::runtime_error(RED "Binding to interface failed: Another program may be using this interface." RESET);
  }

  thread_ = std::thread(&SparkBase::ReadPeriodicMessages, this);
}

SparkBase::~SparkBase()
{
  run_ = false;
  close(soc_);
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void SparkBase::WriteFrame(uint32_t arbId, const uint8_t* data, uint8_t len) const
{
  if (len > 8)
  {
    throw std::runtime_error("CAN frame too large");
  }

  struct can_frame frame = {};
  frame.can_id           = arbId | CAN_EFF_FLAG;
  frame.can_dlc          = len;
  std::memcpy(frame.data, data, len);

  constexpr int MAX_ATTEMPTS = 1000;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt)
  {
    ssize_t bytesSent = write(soc_, &frame, sizeof(frame));
    if (bytesSent == sizeof(frame))
    {
      return;
    }
    if (bytesSent < 0 && (errno == ENOBUFS || errno == EAGAIN))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    char errBuf[128];
    std::snprintf(errBuf, sizeof(errBuf), RED "Failed to send CAN frame: %s" RESET, strerror(errno));
    throw std::runtime_error(errBuf);
  }

  throw std::runtime_error(RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::SendCanFrame(APICommand cmd, const uint8_t* data, uint8_t len) const
{
  WriteFrame(CreateArbId(cmd), data, len);
}

void SparkBase::SendCanFrame(uint32_t arbId, const uint8_t* data, uint8_t len) const
{
  WriteFrame(arbId, data, len);
}

uint8_t SparkBase::GetAPIClass(APICommand cmd)
{
  return static_cast<uint8_t>(static_cast<uint16_t>(cmd) >> 4);
}

uint8_t SparkBase::GetAPIIndex(APICommand cmd)
{
  return static_cast<uint8_t>(static_cast<uint16_t>(cmd) & 0x0F);
}

uint32_t SparkBase::CreateArbId(APICommand cmd) const
{
  uint8_t apiClass = GetAPIClass(cmd);
  uint8_t apiIndex = GetAPIIndex(cmd);

  return (static_cast<uint32_t>(DEVICE_TYPE) << 24) | (static_cast<uint32_t>(MANUFACTURER) << 16) |
         (static_cast<uint32_t>(apiClass) << 10) | (static_cast<uint32_t>(apiIndex) << 6) |
         static_cast<uint32_t>(deviceId_);
}

uint32_t SparkBase::CreateParamArbId(Parameter paramId) const
{
  return (static_cast<uint32_t>(DEVICE_TYPE) << 24) | (static_cast<uint32_t>(MANUFACTURER) << 16) |
         (static_cast<uint32_t>(48) << 10) | (static_cast<uint32_t>(paramId) << 6) | static_cast<uint32_t>(deviceId_);
}

void SparkBase::SendControlMessage(APICommand cmd,
                                   const char* commandName,
                                   float value,
                                   float minValue,
                                   float maxValue) const
{
  if (!std::isfinite(value))
  {
    char errBuf[256];
    std::snprintf(errBuf, sizeof(errBuf), RED "%s must be a finite number." RESET, commandName);
    throw std::invalid_argument(errBuf);
  }
  if (!std::isnan(minValue) && !std::isnan(maxValue) && (value < minValue || value > maxValue))
  {
    char errBuf[256];
    std::snprintf(errBuf,
                  sizeof(errBuf),
                  RED "%s must be between %g and %g" RESET,
                  commandName,
                  static_cast<double>(minValue),
                  static_cast<double>(maxValue));
    throw std::out_of_range(errBuf);
  }

  uint8_t data[8] = {};
  std::memcpy(data, &value, sizeof(value));
  SendCanFrame(cmd, data, 8);
}

void SparkBase::SetParameter(Parameter parameterId,
                             uint8_t parameterType,
                             const char* parameterName,
                             std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
                             float minValue,
                             float maxValue,
                             const char* customErrorMessage)
{
  auto throwRangeError = [&]() {
    if (customErrorMessage)
    {
      char errBuf[512];
      std::snprintf(errBuf, sizeof(errBuf), RED "%s" RESET, customErrorMessage);
      throw std::out_of_range(errBuf);
    }
    else
    {
      char errBuf[256];
      std::snprintf(errBuf,
                    sizeof(errBuf),
                    RED "%s must be between %g and %g" RESET,
                    parameterName,
                    static_cast<double>(minValue),
                    static_cast<double>(maxValue));
      throw std::out_of_range(errBuf);
    }
  };

  uint8_t data[5] = {};
  uint32_t arbId  = CreateParamArbId(parameterId);

  std::visit(
      [&](auto&& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, float>)
        {
          if (!std::isfinite(v))
          {
            char errBuf[256];
            std::snprintf(errBuf, sizeof(errBuf), RED "%s must be a finite number." RESET, parameterName);
            throw std::invalid_argument(errBuf);
          }
          if (!std::isnan(minValue) && !std::isnan(maxValue) && (v < minValue || v > maxValue))
          {
            throwRangeError();
          }
          std::memcpy(data, &v, sizeof(v));
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
          data[0] = v ? 1 : 0;
        }
        else if constexpr (std::is_integral_v<T>)
        {
          std::memcpy(data, &v, sizeof(v));
        }
        else
        {
          throw std::invalid_argument(RED "Unsupported value type." RESET);
        }
      },
      value);

  data[4] = parameterType;
  SendCanFrame(arbId, data, 5);
}

std::optional<std::variant<float, uint32_t, bool>> SparkBase::ReadParameter(Parameter parameterId)
{
  struct can_frame request = {};
  uint32_t requestarbId    = CreateParamArbId(parameterId);
  request.can_id           = requestarbId | CAN_EFF_FLAG;
  request.can_dlc          = 0;

  if (write(soc_, &request, sizeof(request)) < 0)
  {
    return std::nullopt;
  }

  struct can_frame response = {};
  struct timeval tv         = {0, READ_TIMEOUT_US};
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(soc_, &read_fds);

  int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret > 0)
  {
    ssize_t bytesRead = read(soc_, &response, sizeof(response));
    if (bytesRead > 0)
    {
      uint32_t receivedarbId = response.can_id & CAN_EFF_MASK;
      if (receivedarbId == requestarbId)
      {
        uint8_t type = response.data[4];
        switch (type)
        {
          case 0x01: {
            uint32_t val = 0;
            for (int i = 0; i < 4; ++i)
            {
              val |= static_cast<uint32_t>(response.data[i]) << (8 * i);
            }
            return val;
          }
          case 0x02: {
            float val;
            std::memcpy(&val, response.data, sizeof(float));
            return val;
          }
          case 0x03: {
            return static_cast<bool>(response.data[0] != 0);
          }
        }
      }
    }
  }
  return std::nullopt;
}

template <typename T>
T SparkBase::GetParamAs(Parameter param, const char* name)
{
  auto result = ReadParameter(param);
  if (!result.has_value())
  {
    fprintf(stderr, YELLOW "No response for parameter %s, using default value.\n" RESET, name);
    if constexpr (std::is_same_v<T, float>)
    {
      return 0.0f;
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
      return false;
    }
    else
    {
      return static_cast<T>(0);
    }
  }

  if constexpr (std::is_same_v<T, float>)
  {
    if (std::holds_alternative<float>(*result))
    {
      return std::get<float>(*result);
    }
    fprintf(stderr, RED "Wrong type for parameter %s, using default float.\n" RESET, name);
    return 0.0f;
  }
  else if constexpr (std::is_same_v<T, bool>)
  {
    if (std::holds_alternative<bool>(*result))
    {
      return std::get<bool>(*result);
    }
    fprintf(stderr, RED "Wrong type for parameter %s, using default bool.\n" RESET, name);
    return false;
  }
  else if constexpr (std::is_same_v<T, uint8_t>)
  {
    if (std::holds_alternative<uint32_t>(*result))
    {
      return static_cast<uint8_t>(std::get<uint32_t>(*result));
    }
    fprintf(stderr, RED "Wrong type for parameter %s, using default uint8_t.\n" RESET, name);
    return 0;
  }
  else if constexpr (std::is_same_v<T, uint16_t>)
  {
    if (std::holds_alternative<uint32_t>(*result))
    {
      return static_cast<uint16_t>(std::get<uint32_t>(*result));
    }
    fprintf(stderr, RED "Wrong type for parameter %s, using default uint16_t.\n" RESET, name);
    return 0;
  }
  else if constexpr (std::is_same_v<T, uint32_t>)
  {
    if (std::holds_alternative<uint32_t>(*result))
    {
      return std::get<uint32_t>(*result);
    }
    fprintf(stderr, RED "Wrong type for parameter %s, using default uint32_t.\n" RESET, name);
    return 0;
  }
  else
  {
    static_assert(sizeof(T) == 0, "Unsupported type");
  }
}

template <typename T>
T SparkBase::GetSlottedParam(Parameter baseParam, uint8_t stride, uint8_t slot, const char* name)
{
  if (slot >= 4)
  {
    throw std::out_of_range("Invalid slot number");
  }
  return GetParamAs<T>(static_cast<Parameter>(static_cast<int>(baseParam) + slot * stride), name);
}

std::optional<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t, bool>> SparkBase::ReadFirmwareVersion()
{
  struct can_frame request = {};
  uint32_t requestarbId    = CreateArbId(APICommand::FirmwareVersion);
  request.can_id           = requestarbId | CAN_EFF_FLAG;
  request.can_dlc          = 0;

  if (write(soc_, &request, sizeof(request)) < 0)
  {
    return std::nullopt;
  }

  struct can_frame response = {};
  struct timeval tv         = {0, READ_TIMEOUT_US};
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(soc_, &read_fds);

  int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret > 0)
  {
    ssize_t bytesRead = read(soc_, &response, sizeof(response));
    if (bytesRead >= 5)
    {
      uint32_t receivedarbId = response.can_id & CAN_EFF_MASK;
      if (receivedarbId == requestarbId)
      {
        return std::make_tuple(
            response.data[0], response.data[1], response.data[2], response.data[3], response.data[4] != 0);
      }
    }
  }

  return std::nullopt;
}

void SparkBase::ReadPeriodicMessages()
{
  // Set socket to non-blocking
  int flags = fcntl(soc_, F_GETFL, 0);
  fcntl(soc_, F_SETFL, flags | O_NONBLOCK);

  struct can_frame response = {};

  while (run_)
  {
    struct timeval tv = {0, READ_TIMEOUT_US};
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(soc_, &read_fds);

    int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0)
    {
      ssize_t bytesRead = read(soc_, &response, sizeof(response));
      if (bytesRead <= 0)
      {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          break;
        }
        continue;
      }

      uint32_t receivedArbId = response.can_id & CAN_EFF_MASK;
      const uint8_t* d       = response.data;
      auto now               = std::chrono::steady_clock::now();

      std::lock_guard<std::mutex> lock(mutex_);

      if (receivedArbId == CreateArbId(APICommand::Period0))
      {
        period0_.dutyCycle    = unpack<int16_t>(d) / P0_DUTY_CYCLE_SCALE;
        period0_.faults       = unpack<uint16_t>(d, 2);
        period0_.stickyFaults = unpack<uint16_t>(d, 4);
        period0_.isInverted   = (d[6] >> 1) & 1;
        period0_.idleMode     = (d[7] >> 1) & 1;
        period0_.isFollower   = (d[7] >> 2) & 1;
        period0_.timestamp    = now;
      }
      else if (receivedArbId == CreateArbId(APICommand::Period1))
      {
        period1_.velocity    = unpack<float>(d);
        period1_.temperature = d[4];
        period1_.voltage     = unpack<uint16_t>(d, 5) / P1_VOLTAGE_SCALE;
        period1_.current     = (unpack<uint16_t>(d, 6) & 0x0FFF) / P1_CURRENT_SCALE;
        period1_.timestamp   = now;
      }
      else if (receivedArbId == CreateArbId(APICommand::Period2))
      {
        period2_.position  = unpack<float>(d);
        period2_.iAccum    = float(unpack<uint32_t>(d, 4)) / P2_IACCUM_SCALE;
        period2_.timestamp = now;
      }
      else if (receivedArbId == CreateArbId(APICommand::Period3))
      {
        uint16_t v                   = d[0] | (uint16_t(d[1] & 0x03) << 8);
        period3_.analogVoltage       = float(v) / P3_VOLTAGE_SCALE;
        constexpr uint8_t lowerShift = 2;
        constexpr uint8_t midShift   = 8 - lowerShift;
        constexpr uint8_t highShift  = midShift + 8;
        uint32_t vel =
            (uint32_t(d[1] >> lowerShift) & 0x3F) | (uint32_t(d[2]) << midShift) | (uint32_t(d[3]) << highShift);
        period3_.analogVelocity = float(vel) / P3_VELOCITY_SCALE;
        period3_.analogPosition = unpack<float>(d, 4);
        period3_.timestamp      = now;
      }
      else if (receivedArbId == CreateArbId(APICommand::Period4))
      {
        period4_.altEncoderVelocity = unpack<float>(d);
        period4_.altEncoderPosition = unpack<float>(d, 4);
        period4_.timestamp          = now;
      }
    }
  }
}

void SparkBase::Heartbeat()
{
  const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  SendCanFrame(APICommand::Heartbeat, data, 8);
}

void SparkBase::BurnFlash()
{
  const uint8_t data[] = {0xA3, 0x3A};
  SendCanFrame(APICommand::BurnFlash, data, 2);
}

void SparkBase::FactoryDefaults()
{
  const uint8_t data[] = {0x01};
  SendCanFrame(APICommand::FactoryDefaults, data, 1);
}

void SparkBase::FactoryReset()
{
  const uint8_t data[] = {0x01};
  SendCanFrame(APICommand::FactoryReset, data, 1);
}

void SparkBase::Identify()
{
  const uint8_t data[8] = {};
  SendCanFrame(APICommand::Identify, data, 8);
}

void SparkBase::ResetFaults()
{
  const uint8_t data[8] = {};
  SendCanFrame(APICommand::ClearFaults, data, 8);
}

void SparkBase::ClearStickyFaults()
{
  const uint8_t data[8] = {};
  SendCanFrame(APICommand::ClearFaults, data, 8);
}

// //////////////////////////////////////////////////
// MOTOR CONTROL
// //////////////////////////////////////////////////

void SparkBase::SetSetpoint(float setpoint)
{
  SendControlMessage(APICommand::Setpoint, "Setpoint", setpoint);
}

void SparkBase::SetVelocity(float velocity)
{
  SendControlMessage(APICommand::Velocity, "Velocity", velocity);
}

void SparkBase::SetSmartVelocity(float sv)
{
  SendControlMessage(APICommand::SmartVelocity, "Smart Velocity", sv);
}

void SparkBase::SetPosition(float position)
{
  SendControlMessage(APICommand::Position, "Position", position);
}

void SparkBase::SetVoltage(float voltage)
{
  SendControlMessage(APICommand::Voltage, "Voltage", voltage);
}

void SparkBase::SetCurrent(float current)
{
  SendControlMessage(APICommand::Current, "Current", current);
}

void SparkBase::SetSmartMotion(float sm)
{
  SendControlMessage(APICommand::SmartMotion, "Smart Motion", sm);
}

void SparkBase::SetDutyCycle(float dutyCycle)
{
  SendControlMessage(APICommand::DutyCycle, "Duty Cycle", dutyCycle, -1.0f, 1.0f);
}

// //////////////////////////////////////////////////
// STATUS
// //////////////////////////////////////////////////

void SparkBase::SetPeriodicStatusPeriod(APICommand cmd, uint16_t period)
{
  const uint8_t data[2] = {static_cast<uint8_t>(period & 0xFF), static_cast<uint8_t>(period >> 8)};
  SendCanFrame(cmd, data, 2);
}

void SparkBase::SetPeriodicStatus0Period(uint16_t period)
{
  SetPeriodicStatusPeriod(APICommand::Period0, period);
}

void SparkBase::SetPeriodicStatus1Period(uint16_t period)
{
  SetPeriodicStatusPeriod(APICommand::Period1, period);
}

void SparkBase::SetPeriodicStatus2Period(uint16_t period)
{
  SetPeriodicStatusPeriod(APICommand::Period2, period);
}

void SparkBase::SetPeriodicStatus3Period(uint16_t period)
{
  SetPeriodicStatusPeriod(APICommand::Period3, period);
}

void SparkBase::SetPeriodicStatus4Period(uint16_t period)
{
  SetPeriodicStatusPeriod(APICommand::Period4, period);
}

// //////////////////////////////////////////////////
// PERIOD 0
// //////////////////////////////////////////////////

float SparkBase::GetDutyCycle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.dutyCycle;
}

uint16_t SparkBase::GetFaults() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.faults;
}

uint16_t SparkBase::GetStickyFaults() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.stickyFaults;
}

bool SparkBase::IsInverted() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.isInverted;
}

bool SparkBase::GetIdleMode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.idleMode;
}

bool SparkBase::IsFollower() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period0_.isFollower;
}

// //////////////////////////////////////////////////
// PERIOD 1
// //////////////////////////////////////////////////

float SparkBase::GetVelocity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period1_.velocity;
}

float SparkBase::GetTemperature() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period1_.temperature;
}

float SparkBase::GetVoltage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period1_.voltage;
}

float SparkBase::GetCurrent() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period1_.current;
}

// //////////////////////////////////////////////////
// PERIOD 2
// //////////////////////////////////////////////////

float SparkBase::GetPosition() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period2_.position;
}

float SparkBase::GetIAccum() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period2_.iAccum;
}

// //////////////////////////////////////////////////
// PERIOD 3
// //////////////////////////////////////////////////

float SparkBase::GetAnalogVoltage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period3_.analogVoltage;
}

float SparkBase::GetAnalogVelocity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period3_.analogVelocity;
}

float SparkBase::GetAnalogPosition() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period3_.analogPosition;
}

// //////////////////////////////////////////////////
// PERIOD 4
// //////////////////////////////////////////////////

float SparkBase::GetAltEncoderVelocity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period4_.altEncoderVelocity;
}

float SparkBase::GetAltEncoderPosition() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return period4_.altEncoderPosition;
}

// //////////////////////////////////////////////////
// PARAMETER SETTERS
// //////////////////////////////////////////////////

// //////////////////////////////////////////////////
// BASIC
// //////////////////////////////////////////////////

void SparkBase::SetMotorType(MotorType type)
{
  SetParameter(Parameter::kMotorType,
               PARAM_TYPE_UINT,
               "Motor Type",
               static_cast<uint8_t>(type),
               0,
               1,
               "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless).");
}

void SparkBase::SetSensorType(SensorType type)
{
  SetParameter(Parameter::kSensorType,
               PARAM_TYPE_UINT,
               "Sensor Type",
               static_cast<uint8_t>(type),
               0,
               2,
               "Invalid sensor type. Must be 0 (No Sensor), 1 (Hall Sensor), or 2 (Encoder).");
}

void SparkBase::SetIdleMode(IdleMode mode)
{
  SetParameter(Parameter::kIdleMode,
               PARAM_TYPE_UINT,
               "Idle Mode",
               static_cast<uint8_t>(mode),
               0,
               1,
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

// //////////////////////////////////////////////////
// ADVANCED
// //////////////////////////////////////////////////

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

// //////////////////////////////////////////////////
// CLOSED LOOP
// //////////////////////////////////////////////////

void SparkBase::SetCtrlType(CtrlType type)
{
  SetParameter(Parameter::kCtrlType,
               PARAM_TYPE_UINT,
               "Control Type",
               static_cast<uint8_t>(type),
               0,
               3,
               "Invalid control type. Must be 0 (Duty Cycle), 1 (Velocity), 2 (Voltage), or 3 (Position).");
}

void SparkBase::SetFeedbackSensorPID0(uint16_t sensor)
{
  SetParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, "Feedback Sensor PID0", sensor);
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

void SparkBase::SetClosedLoopVoltageMode(uint8_t mode)
{
  SetParameter(Parameter::kClosedLoopVoltageMode,
               PARAM_TYPE_UINT,
               "Closed Loop Voltage Mode",
               mode,
               0,
               2,
               "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control Loop Voltage Output Mode) or 2 "
               "(Voltage Compensation Mode).");
}

// //////////////////////////////////////////////////
// BRUSHLESS
// //////////////////////////////////////////////////

void SparkBase::SetPolePairs(uint16_t pairs)
{
  SetParameter(Parameter::kPolePairs, PARAM_TYPE_UINT, "Pole Pairs", pairs);
}

// //////////////////////////////////////////////////
// CURRENT LIMIT
// //////////////////////////////////////////////////

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

// //////////////////////////////////////////////////
// PIDF
// //////////////////////////////////////////////////

void SparkBase::SetSlottedParam(const Parameter params[4],
                                uint8_t slot,
                                uint8_t paramType,
                                const char* name,
                                float value)
{
  if (slot >= 4)
  {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }
  SetParameter(params[slot], paramType, name, value);
}

static const Parameter kParamsP[4] = {Parameter::kP_0, Parameter::kP_1, Parameter::kP_2, Parameter::kP_3};
static const Parameter kParamsI[4] = {Parameter::kI_0, Parameter::kI_1, Parameter::kI_2, Parameter::kI_3};
static const Parameter kParamsD[4] = {Parameter::kD_0, Parameter::kD_1, Parameter::kD_2, Parameter::kD_3};
static const Parameter kParamsF[4] = {Parameter::kF_0, Parameter::kF_1, Parameter::kF_2, Parameter::kF_3};

static const Parameter kParamsIZone[4] = {Parameter::kIZone_0,
                                          Parameter::kIZone_1,
                                          Parameter::kIZone_2,
                                          Parameter::kIZone_3};

static const Parameter kParamsDFilter[4] = {Parameter::kDFilter_0,
                                            Parameter::kDFilter_1,
                                            Parameter::kDFilter_2,
                                            Parameter::kDFilter_3};

static const Parameter kParamsOutMin[4] = {Parameter::kOutputMin_0,
                                           Parameter::kOutputMin_1,
                                           Parameter::kOutputMin_2,
                                           Parameter::kOutputMin_3};

static const Parameter kParamsOutMax[4] = {Parameter::kOutputMax_0,
                                           Parameter::kOutputMax_1,
                                           Parameter::kOutputMax_2,
                                           Parameter::kOutputMax_3};

void SparkBase::SetP(uint8_t slot, float p)
{
  SetSlottedParam(kParamsP, slot, PARAM_TYPE_FLOAT, "P", p);
}

void SparkBase::SetI(uint8_t slot, float i)
{
  SetSlottedParam(kParamsI, slot, PARAM_TYPE_FLOAT, "I", i);
}

void SparkBase::SetD(uint8_t slot, float d)
{
  SetSlottedParam(kParamsD, slot, PARAM_TYPE_FLOAT, "D", d);
}

void SparkBase::SetF(uint8_t slot, float f)
{
  SetSlottedParam(kParamsF, slot, PARAM_TYPE_FLOAT, "F", f);
}

void SparkBase::SetIZone(uint8_t slot, float iz)
{
  SetSlottedParam(kParamsIZone, slot, PARAM_TYPE_FLOAT, "IZone", iz);
}

void SparkBase::SetDFilter(uint8_t slot, float df)
{
  SetSlottedParam(kParamsDFilter, slot, PARAM_TYPE_FLOAT, "DFilter", df);
}

void SparkBase::SetOutputMin(uint8_t slot, float min)
{
  SetSlottedParam(kParamsOutMin, slot, PARAM_TYPE_FLOAT, "Output Min", min);
}

void SparkBase::SetOutputMax(uint8_t slot, float max)
{
  SetSlottedParam(kParamsOutMax, slot, PARAM_TYPE_FLOAT, "Output Max", max);
}

// //////////////////////////////////////////////////
// LIMITS
// //////////////////////////////////////////////////

void SparkBase::SetHardLimitFwdEn(bool enable)
{
  SetParameter(Parameter::kHardLimitFwdEn, PARAM_TYPE_BOOL, "Hard Limit Forward Enable", enable);
}

void SparkBase::SetHardLimitRevEn(bool enable)
{
  SetParameter(Parameter::kHardLimitRevEn, PARAM_TYPE_BOOL, "Hard Limit Reverse Enable", enable);
}

void SparkBase::SetLimitSwitchFwdPolarity(bool pol)
{
  SetParameter(Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, "Limit Switch Forward Polarity", pol);
}

void SparkBase::SetLimitSwitchRevPolarity(bool pol)
{
  SetParameter(Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, "Limit Switch Reverse Polarity", pol);
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

// //////////////////////////////////////////////////
// FOLLOWER
// //////////////////////////////////////////////////

void SparkBase::SetFollowerID(uint32_t id)
{
  SetParameter(Parameter::kFollowerID, PARAM_TYPE_UINT, "Follower ID", id);
}

void SparkBase::SetFollowerConfig(uint32_t config)
{
  SetParameter(Parameter::kFollowerConfig, PARAM_TYPE_UINT, "Follower Config", config);
}

// //////////////////////////////////////////////////
// ENCODER PORT
// //////////////////////////////////////////////////

void SparkBase::SetEncoderCountsPerRev(uint16_t counts)
{
  SetParameter(Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, "Encoder Counts Per Revolution", counts);
}

void SparkBase::SetEncoderInverted(bool inverted)
{
  SetParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, "Encoder Inverted", inverted);
}

void SparkBase::SetPositionConversionFactor(float f)
{
  SetParameter(Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, "Position Conversion Factor", f);
}

void SparkBase::SetVelocityConversionFactor(float f)
{
  SetParameter(Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, "Velocity Conversion Factor", f);
}

void SparkBase::SetClosedLoopRampRate(float rampRate)
{
  SetParameter(Parameter::kClosedLoopRampRate, PARAM_TYPE_FLOAT, "Closed Loop Ramp Rate", rampRate);
}

void SparkBase::SetHallSensorSampleRate(float rate)
{
  SetParameter(Parameter::kHallSensorSampleRate, PARAM_TYPE_FLOAT, "Hall Sensor Sample Rate", rate);
}

void SparkBase::SetHallSensorAverageDepth(uint16_t d)
{
  SetParameter(Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, "Hall Sensor Average Depth", d);
}

void SparkBase::SetEncoderAverageDepth(uint8_t depth)
{
  SetParameter(Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, "Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetEncoderSampleDelta(uint8_t delta)
{
  SetParameter(Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, "Encoder Sample Delta", delta, 1, 255);
}

// //////////////////////////////////////////////////
// SMART MOTION
// //////////////////////////////////////////////////

static const Parameter kParamsSmMaxVel[4] = {Parameter::kSmartMotionMaxVelocity_0,
                                             Parameter::kSmartMotionMaxVelocity_1,
                                             Parameter::kSmartMotionMaxVelocity_2,
                                             Parameter::kSmartMotionMaxVelocity_3};

static const Parameter kParamsSmMaxAccel[4] = {Parameter::kSmartMotionMaxAccel_0,
                                               Parameter::kSmartMotionMaxAccel_1,
                                               Parameter::kSmartMotionMaxAccel_2,
                                               Parameter::kSmartMotionMaxAccel_3};

static const Parameter kParamsSmMinVel[4] = {Parameter::kSmartMotionMinVelOutput_0,
                                             Parameter::kSmartMotionMinVelOutput_1,
                                             Parameter::kSmartMotionMinVelOutput_2,
                                             Parameter::kSmartMotionMinVelOutput_3};

static const Parameter kParamsSmErr[4] = {Parameter::kSmartMotionAllowedClosedLoopError_0,
                                          Parameter::kSmartMotionAllowedClosedLoopError_1,
                                          Parameter::kSmartMotionAllowedClosedLoopError_2,
                                          Parameter::kSmartMotionAllowedClosedLoopError_3};

static const Parameter kParamsSmStrat[4] = {Parameter::kSmartMotionAccelStrategy_0,
                                            Parameter::kSmartMotionAccelStrategy_1,
                                            Parameter::kSmartMotionAccelStrategy_2,
                                            Parameter::kSmartMotionAccelStrategy_3};

static const Parameter kParamsIMaxAccum[4] = {Parameter::kIMaxAccum_0,
                                              Parameter::kIMaxAccum_1,
                                              Parameter::kIMaxAccum_2,
                                              Parameter::kIMaxAccum_3};

static const Parameter kParamsSlot3P1[4] = {Parameter::kSlot3Placeholder1_0,
                                            Parameter::kSlot3Placeholder1_1,
                                            Parameter::kSlot3Placeholder1_2,
                                            Parameter::kSlot3Placeholder1_3};

static const Parameter kParamsSlot3P2[4] = {Parameter::kSlot3Placeholder2_0,
                                            Parameter::kSlot3Placeholder2_1,
                                            Parameter::kSlot3Placeholder2_2,
                                            Parameter::kSlot3Placeholder2_3};

static const Parameter kParamsSlot3P3[4] = {Parameter::kSlot3Placeholder3_0,
                                            Parameter::kSlot3Placeholder3_1,
                                            Parameter::kSlot3Placeholder3_2,
                                            Parameter::kSlot3Placeholder3_3};

void SparkBase::SetSmartMotionMaxVelocity(uint8_t slot, float v)
{
  SetSlottedParam(kParamsSmMaxVel, slot, PARAM_TYPE_FLOAT, "Smart Motion Max Velocity", v);
}

void SparkBase::SetSmartMotionMaxAccel(uint8_t slot, float a)
{
  SetSlottedParam(kParamsSmMaxAccel, slot, PARAM_TYPE_FLOAT, "Smart Motion Max Accel", a);
}

void SparkBase::SetSmartMotionMinVelOutput(uint8_t slot, float v)
{
  SetSlottedParam(kParamsSmMinVel, slot, PARAM_TYPE_FLOAT, "Smart Motion Min Vel Output", v);
}

void SparkBase::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float e)
{
  SetSlottedParam(kParamsSmErr, slot, PARAM_TYPE_FLOAT, "Smart Motion Allowed Closed Loop Error", e);
}

void SparkBase::SetSmartMotionAccelStrategy(uint8_t slot, float s)
{
  SetSlottedParam(kParamsSmStrat, slot, PARAM_TYPE_FLOAT, "Smart Motion Accel Strategy", s);
}

void SparkBase::SetIMaxAccum(uint8_t slot, float m)
{
  SetSlottedParam(kParamsIMaxAccum, slot, PARAM_TYPE_FLOAT, "IMaxAccum", m);
}

void SparkBase::SetSlot3Placeholder1(uint8_t slot, float v)
{
  SetSlottedParam(kParamsSlot3P1, slot, PARAM_TYPE_FLOAT, "Slot 3 Placeholder 1", v);
}

void SparkBase::SetSlot3Placeholder2(uint8_t slot, float v)
{
  SetSlottedParam(kParamsSlot3P2, slot, PARAM_TYPE_FLOAT, "Slot 3 Placeholder 2", v);
}

void SparkBase::SetSlot3Placeholder3(uint8_t slot, float v)
{
  SetSlottedParam(kParamsSlot3P3, slot, PARAM_TYPE_FLOAT, "Slot 3 Placeholder 3", v);
}

// //////////////////////////////////////////////////
// ANALOG SENSOR
// //////////////////////////////////////////////////

void SparkBase::SetAnalogPositionConversion(float f)
{
  SetParameter(Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, "Analog Position Conversion", f);
}

void SparkBase::SetAnalogVelocityConversion(float f)
{
  SetParameter(Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, "Analog Velocity Conversion", f);
}

void SparkBase::SetAnalogAverageDepth(uint16_t d)
{
  SetParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, "Analog Average Depth", d);
}

void SparkBase::SetAnalogInverted(bool inverted)
{
  SetParameter(Parameter::kAnalogInverted, PARAM_TYPE_BOOL, "Analog Inverted", inverted);
}

void SparkBase::SetAnalogSampleDelta(uint16_t delta)
{
  SetParameter(Parameter::kAnalogSampleDelta, PARAM_TYPE_UINT, "Analog Sample Delta", delta);
}

void SparkBase::SetAnalogSensorMode(uint8_t mode)
{
  SetParameter(Parameter::kAnalogSensorMode,
               PARAM_TYPE_UINT,
               "Analog Sensor Mode",
               mode,
               0,
               1,
               "Invalid analog sensor mode. Must be 0 (Absolute) or 1 (Relative).");
}

// //////////////////////////////////////////////////
// ALTERNATE ENCODER
// //////////////////////////////////////////////////

void SparkBase::SetAltEncoderCountsPerRev(uint16_t counts)
{
  SetParameter(Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, "Alternate Encoder Counts Per Revolution", counts);
}

void SparkBase::SetAltEncoderInverted(bool inverted)
{
  SetParameter(Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, "Alternate Encoder Inverted", inverted);
}

void SparkBase::SetAltEncoderPositionFactor(float f)
{
  SetParameter(Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Position Factor", f);
}

void SparkBase::SetAltEncoderVelocityFactor(float f)
{
  SetParameter(Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Velocity Factor", f);
}

void SparkBase::SetAltEncoderAverageDepth(uint8_t depth)
{
  SetParameter(Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT, "Alternate Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetAltEncoderSampleDelta(uint8_t delta)
{
  SetParameter(Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, "Alternate Encoder Sample Delta", delta, 1, 255);
}

void SparkBase::SetDataPortConfig(uint8_t config)
{
  SetParameter(Parameter::kDataPortConfig,
               PARAM_TYPE_UINT,
               "Data Port Config",
               config,
               0,
               1,
               "Invalid data port config. Must be 0 (Default) or 1 (Alternate Encoder Mode).");
}

// //////////////////////////////////////////////////
// DUTY CYCLE ABSOLUTE ENCODER
// //////////////////////////////////////////////////

void SparkBase::SetDutyCyclePositionFactor(float f)
{
  SetParameter(Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, "Duty Cycle Position Factor", f);
}

void SparkBase::SetDutyCycleVelocityFactor(float f)
{
  SetParameter(Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, "Duty Cycle Velocity Factor", f);
}

void SparkBase::SetDutyCycleInverted(bool inverted)
{
  SetParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, "Duty Cycle Inverted", inverted);
}

void SparkBase::SetDutyCycleZeroOffset(float offset)
{
  SetParameter(Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, "Duty Cycle Zero Offset", offset, 0.0f, 1.0f);
}

void SparkBase::SetDutyCycleAverageDepth(uint8_t depth)
{
  SetParameter(Parameter::kDutyCycleAverageDepth,
               PARAM_TYPE_UINT,
               "Duty Cycle Average Depth",
               depth,
               0,
               7,
               "Invalid average depth. Must be 0 (1 bit), 2 (2 bits), 3 (4 bits), 4 (8 bits), 5 (16 bits), 6 (32 "
               "bits), or 7 (64 bits).");
}

void SparkBase::SetDutyCyclePrescalar(uint8_t prescalar)
{
  SetParameter(Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, "Duty Cycle Prescalar", prescalar, 0, 71);
}

// //////////////////////////////////////////////////
// PARAMETER GETTERS
// //////////////////////////////////////////////////

// //////////////////////////////////////////////////
// BASIC
// //////////////////////////////////////////////////

uint8_t SparkBase::GetMotorType()
{
  return GetParamAs<uint8_t>(Parameter::kMotorType, "MotorType");
}

uint8_t SparkBase::GetSensorType()
{
  return GetParamAs<uint8_t>(Parameter::kSensorType, "SensorType");
}

uint8_t SparkBase::GetIdleMode()
{
  return GetParamAs<uint8_t>(Parameter::kIdleMode, "IdleMode");
}

float SparkBase::GetInputDeadband()
{
  return GetParamAs<float>(Parameter::kInputDeadband, "InputDeadband");
}

bool SparkBase::GetInverted()
{
  return GetParamAs<bool>(Parameter::kInverted, "Inverted");
}

float SparkBase::GetRampRate()
{
  return GetParamAs<float>(Parameter::kRampRate, "RampRate");
}

// //////////////////////////////////////////////////
// ADVANCED
// //////////////////////////////////////////////////

uint16_t SparkBase::GetMotorKv()
{
  return GetParamAs<uint16_t>(Parameter::kMotorKv, "MotorKv");
}

uint16_t SparkBase::GetMotorR()
{
  return GetParamAs<uint16_t>(Parameter::kMotorR, "MotorR");
}

uint16_t SparkBase::GetMotorL()
{
  return GetParamAs<uint16_t>(Parameter::kMotorL, "MotorL");
}

// //////////////////////////////////////////////////
// CLOSED LOOP
// //////////////////////////////////////////////////

uint8_t SparkBase::GetCtrlType()
{
  return GetParamAs<uint8_t>(Parameter::kCtrlType, "CtrlType");
}

uint16_t SparkBase::GetFeedbackSensorPID0()
{
  return GetParamAs<uint16_t>(Parameter::kFeedbackSensorPID0, "FeedbackSensorPID0");
}

uint8_t SparkBase::GetClosedLoopVoltageMode()
{
  return GetParamAs<uint8_t>(Parameter::kClosedLoopVoltageMode, "ClosedLoopVoltageMode");
}

float SparkBase::GetCompensatedNominalVoltage()
{
  return GetParamAs<float>(Parameter::kCompensatedNominalVoltage, "CompensatedNominalVoltage");
}

bool SparkBase::GetPositionPIDWrapEnable()
{
  return GetParamAs<bool>(Parameter::kPositionPIDWrapEnable, "PositionPIDWrapEnable");
}

float SparkBase::GetPositionPIDMinInput()
{
  return GetParamAs<float>(Parameter::kPositionPIDMinInput, "PositionPIDMinInput");
}

float SparkBase::GetPositionPIDMaxInput()
{
  return GetParamAs<float>(Parameter::kPositionPIDMaxInput, "PositionPIDMaxInput");
}

// //////////////////////////////////////////////////
// BRUSHLESS
// //////////////////////////////////////////////////

uint16_t SparkBase::GetPolePairs()
{
  return GetParamAs<uint16_t>(Parameter::kPolePairs, "PolePairs");
}

// //////////////////////////////////////////////////
// CURRENT LIMIT
// //////////////////////////////////////////////////

float SparkBase::GetCurrentChop()
{
  return GetParamAs<float>(Parameter::kCurrentChop, "CurrentChop");
}

uint16_t SparkBase::GetCurrentChopCycles()
{
  return GetParamAs<uint16_t>(Parameter::kCurrentChopCycles, "CurrentChopCycles");
}

uint16_t SparkBase::GetSmartCurrentStallLimit()
{
  return GetParamAs<uint16_t>(Parameter::kSmartCurrentStallLimit, "SmartCurrentStallLimit");
}

uint16_t SparkBase::GetSmartCurrentFreeLimit()
{
  return GetParamAs<uint16_t>(Parameter::kSmartCurrentFreeLimit, "SmartCurrentFreeLimit");
}

uint16_t SparkBase::GetSmartCurrentConfig()
{
  return GetParamAs<uint16_t>(Parameter::kSmartCurrentConfig, "SmartCurrentConfig");
}

// //////////////////////////////////////////////////
// PIDF
// //////////////////////////////////////////////////

float SparkBase::GetP(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kP_0, 8, slot, "P");
}

float SparkBase::GetI(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kI_0, 8, slot, "I");
}

float SparkBase::GetD(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kD_0, 8, slot, "D");
}

float SparkBase::GetF(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kF_0, 8, slot, "F");
}

float SparkBase::GetIZone(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kIZone_0, 8, slot, "IZone");
}

float SparkBase::GetDFilter(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kDFilter_0, 8, slot, "DFilter");
}

float SparkBase::GetOutputMin(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kOutputMin_0, 8, slot, "OutputMin");
}

float SparkBase::GetOutputMax(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kOutputMax_0, 8, slot, "OutputMax");
}

// //////////////////////////////////////////////////
// LIMITS
// //////////////////////////////////////////////////

bool SparkBase::GetHardLimitFwdEn()
{
  return GetParamAs<bool>(Parameter::kHardLimitFwdEn, "HardLimitFwdEn");
}

bool SparkBase::GetHardLimitRevEn()
{
  return GetParamAs<bool>(Parameter::kHardLimitRevEn, "HardLimitRevEn");
}

bool SparkBase::GetLimitSwitchFwdPolarity()
{
  return GetParamAs<bool>(Parameter::kLimitSwitchFwdPolarity, "LimitSwitchFwdPolarity");
}

bool SparkBase::GetLimitSwitchRevPolarity()
{
  return GetParamAs<bool>(Parameter::kLimitSwitchRevPolarity, "LimitSwitchRevPolarity");
}

bool SparkBase::GetSoftLimitFwdEn()
{
  return GetParamAs<bool>(Parameter::kSoftLimitFwdEn, "SoftLimitFwdEn");
}

bool SparkBase::GetSoftLimitRevEn()
{
  return GetParamAs<bool>(Parameter::kSoftLimitRevEn, "SoftLimitRevEn");
}

float SparkBase::GetSoftLimitFwd()
{
  return GetParamAs<float>(Parameter::kSoftLimitFwd, "SoftLimitFwd");
}

float SparkBase::GetSoftLimitRev()
{
  return GetParamAs<float>(Parameter::kSoftLimitRev, "SoftLimitRev");
}

// //////////////////////////////////////////////////
// FOLLOWER
// //////////////////////////////////////////////////

uint32_t SparkBase::GetFollowerID()
{
  return GetParamAs<uint32_t>(Parameter::kFollowerID, "FollowerID");
}

uint32_t SparkBase::GetFollowerConfig()
{
  return GetParamAs<uint32_t>(Parameter::kFollowerConfig, "FollowerConfig");
}

// //////////////////////////////////////////////////
// ENCODER PORT
// //////////////////////////////////////////////////

uint16_t SparkBase::GetEncoderCountsPerRev()
{
  return GetParamAs<uint16_t>(Parameter::kEncoderCountsPerRev, "EncoderCountsPerRev");
}

uint8_t SparkBase::GetEncoderAverageDepth()
{
  return GetParamAs<uint8_t>(Parameter::kEncoderAverageDepth, "EncoderAverageDepth");
}

uint8_t SparkBase::GetEncoderSampleDelta()
{
  return GetParamAs<uint8_t>(Parameter::kEncoderSampleDelta, "EncoderSampleDelta");
}

bool SparkBase::GetEncoderInverted()
{
  return GetParamAs<bool>(Parameter::kEncoderInverted, "EncoderInverted");
}

float SparkBase::GetPositionConversionFactor()
{
  return GetParamAs<float>(Parameter::kPositionConversionFactor, "PositionConversionFactor");
}

float SparkBase::GetVelocityConversionFactor()
{
  return GetParamAs<float>(Parameter::kVelocityConversionFactor, "VelocityConversionFactor");
}

float SparkBase::GetClosedLoopRampRate()
{
  return GetParamAs<float>(Parameter::kClosedLoopRampRate, "ClosedLoopRampRate");
}

float SparkBase::GetHallSensorSampleRate()
{
  return GetParamAs<float>(Parameter::kHallSensorSampleRate, "HallSensorSampleRate");
}

uint16_t SparkBase::GetHallSensorAverageDepth()
{
  return GetParamAs<uint16_t>(Parameter::kHallSensorAverageDepth, "HallSensorAverageDepth");
}

// //////////////////////////////////////////////////
// SMART MOTION
// //////////////////////////////////////////////////

float SparkBase::GetSmartMotionMaxVelocity(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSmartMotionMaxVelocity_0, 5, slot, "SmartMotionMaxVelocity");
}

float SparkBase::GetSmartMotionMaxAccel(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSmartMotionMaxAccel_0, 5, slot, "SmartMotionMaxAccel");
}

float SparkBase::GetSmartMotionMinVelOutput(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSmartMotionMinVelOutput_0, 5, slot, "SmartMotionMinVelOutput");
}

float SparkBase::GetSmartMotionAllowedClosedLoopError(uint8_t slot)
{
  return GetSlottedParam<float>(
      Parameter::kSmartMotionAllowedClosedLoopError_0, 5, slot, "SmartMotionAllowedClosedLoopError");
}

float SparkBase::GetSmartMotionAccelStrategy(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSmartMotionAccelStrategy_0, 5, slot, "SmartMotionAccelStrategy");
}

float SparkBase::GetIMaxAccum(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kIMaxAccum_0, 4, slot, "IMaxAccum");
}

float SparkBase::GetSlot3Placeholder1(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSlot3Placeholder1_0, 4, slot, "Slot3Placeholder1");
}

float SparkBase::GetSlot3Placeholder2(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSlot3Placeholder2_0, 4, slot, "Slot3Placeholder2");
}

float SparkBase::GetSlot3Placeholder3(uint8_t slot)
{
  return GetSlottedParam<float>(Parameter::kSlot3Placeholder3_0, 4, slot, "Slot3Placeholder3");
}

// //////////////////////////////////////////////////
// ANALOG SENSOR
// //////////////////////////////////////////////////

float SparkBase::GetAnalogPositionConversion()
{
  return GetParamAs<float>(Parameter::kAnalogPositionConversion, "AnalogPositionConversion");
}

float SparkBase::GetAnalogVelocityConversion()
{
  return GetParamAs<float>(Parameter::kAnalogVelocityConversion, "AnalogVelocityConversion");
}

uint16_t SparkBase::GetAnalogAverageDepth()
{
  return GetParamAs<uint16_t>(Parameter::kAnalogAverageDepth, "AnalogAverageDepth");
}

uint8_t SparkBase::GetAnalogSensorMode()
{
  return GetParamAs<uint8_t>(Parameter::kAnalogSensorMode, "AnalogSensorMode");
}

bool SparkBase::GetAnalogInverted()
{
  return GetParamAs<bool>(Parameter::kAnalogInverted, "AnalogInverted");
}

uint16_t SparkBase::GetAnalogSampleDelta()
{
  return GetParamAs<uint16_t>(Parameter::kAnalogSampleDelta, "AnalogSampleDelta");
}

// //////////////////////////////////////////////////
// ALTERNATE ENCODER
// //////////////////////////////////////////////////

uint8_t SparkBase::GetDataPortConfig()
{
  return GetParamAs<uint8_t>(Parameter::kDataPortConfig, "DataPortConfig");
}

uint16_t SparkBase::GetAltEncoderCountsPerRev()
{
  return GetParamAs<uint16_t>(Parameter::kAltEncoderCountsPerRev, "AltEncoderCountsPerRev");
}

uint8_t SparkBase::GetAltEncoderAverageDepth()
{
  return GetParamAs<uint8_t>(Parameter::kAltEncoderAverageDepth, "AltEncoderAverageDepth");
}

uint8_t SparkBase::GetAltEncoderSampleDelta()
{
  return GetParamAs<uint8_t>(Parameter::kAltEncoderSampleDelta, "AltEncoderSampleDelta");
}

bool SparkBase::GetAltEncoderInverted()
{
  return GetParamAs<bool>(Parameter::kAltEncoderInverted, "AltEncoderInverted");
}

float SparkBase::GetAltEncoderPositionFactor()
{
  return GetParamAs<float>(Parameter::kAltEncoderPositionFactor, "AltEncoderPositionFactor");
}

float SparkBase::GetAltEncoderVelocityFactor()
{
  return GetParamAs<float>(Parameter::kAltEncoderVelocityFactor, "AltEncoderVelocityFactor");
}

// //////////////////////////////////////////////////
// DUTY CYCLE ABSOLUTE ENCODER
// //////////////////////////////////////////////////

float SparkBase::GetDutyCyclePositionFactor()
{
  return GetParamAs<float>(Parameter::kDutyCyclePositionFactor, "DutyCyclePositionFactor");
}

float SparkBase::GetDutyCycleVelocityFactor()
{
  return GetParamAs<float>(Parameter::kDutyCycleVelocityFactor, "DutyCycleVelocityFactor");
}

bool SparkBase::GetDutyCycleInverted()
{
  return GetParamAs<bool>(Parameter::kDutyCycleInverted, "DutyCycleInverted");
}

uint8_t SparkBase::GetDutyCycleAverageDepth()
{
  return GetParamAs<uint8_t>(Parameter::kDutyCycleAverageDepth, "DutyCycleAverageDepth");
}

uint8_t SparkBase::GetDutyCyclePrescalar()
{
  return GetParamAs<uint8_t>(Parameter::kDutyCyclePrescalar, "DutyCyclePrescalar");
}

float SparkBase::GetDutyCycleZeroOffset()
{
  return GetParamAs<float>(Parameter::kDutyCycleZeroOffset, "DutyCycleZeroOffset");
}
