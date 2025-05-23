/**
 * @file SparkBase.cpp
 * @brief Source file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#include "SparkBase.hpp"

SparkBase::SparkBase(const std::string & interfaceName, uint8_t deviceId)
: interfaceName_(interfaceName), deviceId_(deviceId)
{
  // Ensure deviceId_ is within valid range
  if (deviceId_ > 62) {
    throw std::out_of_range(RED "Invalid CAN bus ID. Must be between 0 and 62." RESET);
  }

  // Create CAN socket
  SparkBase::soc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (SparkBase::soc_ < 0) {
    throw std::system_error(
            errno, std::generic_category(),
            std::string(RED) + "Socket creation failed: " + strerror(errno) +
            "\nPossible causes:\n"
            "1. CAN modules not loaded\n"
            "2. System resource limitations" +
            std::string(RESET));
  }

  // Prepare CAN interface request structure
  std::memset(&ifr_, 0, sizeof(ifr_));
  std::strncpy(ifr_.ifr_name, interfaceName_.c_str(), sizeof(ifr_.ifr_name) - 1);

  // Get CAN interface index
  if (ioctl(soc_, SIOCGIFINDEX, &ifr_) < 0) {
    close(soc_);
    throw std::runtime_error(
            std::string(RED) + "IOCTL failed: " + strerror(errno) +
            "\nPossible causes:\n1. CAN interface does not exist\n2. CAN bus not initialized\n3. "
            "CAN interface is not up" +
            std::string(RESET));
  }

  // Set up CAN address and bind socket to interface
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  if (bind(soc_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    close(soc_);
    throw std::runtime_error(
            RED "Binding to interface failed: Another program may be using this interface." RESET);
  }

  thread_ = std::thread(&SparkBase::ReadPeriodicMessages, this);
}

SparkBase::~SparkBase()
{
  run_ = false;
  close(soc_);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SparkBase::SendCanFrame(APICommand cmd, const std::vector<uint8_t> & data) const
{
  if (data.size() > 8) {
    throw std::runtime_error("CAN frame too large");
  }

  struct can_frame frame = {};
  frame.can_id = CreateArbId(cmd) | CAN_EFF_FLAG;
  frame.can_dlc = static_cast<uint8_t>(data.size());
  std::memcpy(frame.data, data.data(), data.size());

  constexpr std::chrono::milliseconds WAIT_TIME(1);
  constexpr int MAX_ATTEMPTS = 1000;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
    ssize_t bytesSent = write(soc_, &frame, sizeof(frame));
    if (bytesSent == sizeof(frame)) {
      return;
    }

    if (bytesSent < 0 && (errno == ENOBUFS || errno == EAGAIN)) {
      std::this_thread::sleep_for(WAIT_TIME);
      continue;
    }

    throw std::runtime_error(
            RED "Failed to send CAN frame: " + std::string(strerror(errno)) +
            RESET);
  }

  throw std::runtime_error(RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::SendCanFrame(uint32_t arbId, const std::vector<uint8_t> & data) const
{
  if (data.size() > 8) {
    throw std::runtime_error("CAN frame too large");
  }

  struct can_frame frame = {};
  frame.can_id = arbId | CAN_EFF_FLAG;
  frame.can_dlc = static_cast<uint8_t>(data.size());
  std::memcpy(frame.data, data.data(), data.size());

  constexpr std::chrono::milliseconds WAIT_TIME(1);
  constexpr int MAX_ATTEMPTS = 1000;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
    ssize_t bytesSent = write(soc_, &frame, sizeof(frame));
    if (bytesSent == sizeof(frame)) {
      return;
    }
    if (bytesSent < 0 && (errno == ENOBUFS || errno == EAGAIN)) {
      std::this_thread::sleep_for(WAIT_TIME);
      continue;
    }
    throw std::runtime_error("Failed to send CAN frame: " + std::string(strerror(errno)));
  }

  throw std::runtime_error("Failed to send CAN frame: Buffer consistently full.");
}

uint8_t SparkBase::GetAPIClass(APICommand cmd) const
{
  return static_cast<uint8_t>(static_cast<uint16_t>(cmd) >> 4);
}

uint8_t SparkBase::GetAPIIndex(APICommand cmd) const
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
         (static_cast<uint32_t>(48) <<
         10) | (static_cast<uint32_t>(paramId) << 6) | static_cast<uint32_t>(deviceId_);
}

void SparkBase::SendControlMessage(
  APICommand cmd, std::string commandName, float value, std::optional<float> minValue,
  std::optional<float> maxValue) const
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(RED + commandName + " must be a finite number." + RESET);
  }
  if (minValue && maxValue && (value < *minValue || value > *maxValue)) {
    throw std::out_of_range(
            RED + commandName + " must be between " + std::to_string(*minValue) + " and " +
            std::to_string(*maxValue) + RESET);
  }

  std::vector<uint8_t> data(8, 0);
  std::memcpy(data.data(), &value, sizeof(value));
  SendCanFrame(cmd, data);
}

void SparkBase::SetParameter(
  Parameter parameterId, uint8_t parameterType, std::string parameterName,
  std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
  std::optional<float> minValue, std::optional<float> maxValue,
  std::optional<std::string> customErrorMessage)
{
  // Lambda to handle range validation errors
  auto throwRangeError = [&](auto min, auto max) {
      if (customErrorMessage) {
        throw std::out_of_range(RED + *customErrorMessage + RESET);
      } else {
        throw std::out_of_range(
                RED + parameterName + " must be between " + std::to_string(min) + " and " +
                std::to_string(max) + RESET);
      }
    };

  // Create CAN data and arbitration ID
  std::vector<uint8_t> data(5, 0);
  uint32_t arbId = CreateParamArbId(parameterId);

  // Process the value based on its type and fill CAN data
  std::visit(
    [&](auto && v) {
      using T = std::decay_t<decltype(v)>;
      if constexpr (std::is_same_v<T, float>) {
        if (!std::isfinite(v)) { // Ensure float is valid
          throw std::invalid_argument(RED + parameterName + " must be a finite number." + RESET);
        }
        if (minValue && maxValue && (v < minValue.value() || v > maxValue.value())) {
          throwRangeError(minValue.value(), maxValue.value());
        }
        std::memcpy(data.data(), &v, sizeof(v));         // Copy float to CAN data
      } else if constexpr (std::is_integral_v<T>) {
        std::memcpy(data.data(), &v, sizeof(v));         // Copy integer to CAN data
      } else if constexpr (std::is_same_v<T, bool>) {
        data[0] = v ? 1 : 0;         // Handle boolean type
      } else {
        throw std::invalid_argument(RED "Unsupported value type." RESET);
      }
    },
    value);

  data[4] = parameterType;     // Add parameter type to CAN data
  SendCanFrame(arbId, data);   // Send CAN frame with parameter data
}

std::optional<std::variant<float, uint32_t, bool>> SparkBase::ReadParameter(Parameter parameterId)
{
  struct can_frame request = {};
  uint32_t requestarbId = CreateParamArbId(parameterId);
  request.can_id = requestarbId | CAN_EFF_FLAG;
  request.can_dlc = 0;

  if (write(soc_, &request, sizeof(request)) < 0) {
    perror("Error sending CAN message");
    return std::nullopt;
  }

  struct can_frame response = {};
  struct timeval tv = {0, READ_TIMEOUT_US};
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(soc_, &read_fds);

  int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret > 0) {
    ssize_t bytesRead = read(soc_, &response, sizeof(response));
    if (bytesRead > 0) {
      uint32_t receivedarbId = response.can_id & CAN_EFF_MASK;
      if (receivedarbId == requestarbId) {
        uint8_t type = response.data[4];
        switch (type) {
          case 0x01: {
              uint32_t val = 0;
              for (int i = 0; i < 4; ++i) {
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

template<typename T> T SparkBase::GetParamAs(Parameter param, const char * name)
{
  auto result = ReadParameter(param);
  if (!result.has_value()) {
    std::cerr << YELLOW << "No response for parameter " << name << ", using default value.\n" <<
      RESET;
    if constexpr (std::is_same_v<T, float>) {
      return 0.0f;
    } else if constexpr (std::is_same_v<T, bool>) {
      return false;
    } else {
      return static_cast<T>(0);
    }
  }

  if constexpr (std::is_same_v<T, float>) {
    if (std::holds_alternative<float>(*result)) {
      return std::get<float>(*result);
    }
    std::cerr << RED << "Wrong type for parameter " << name << ", using default float.\n" << RESET;
    return 0.0f;
  } else if constexpr (std::is_same_v<T, bool>) {
    if (std::holds_alternative<bool>(*result)) {
      return std::get<bool>(*result);
    }
    std::cerr << RED << "Wrong type for parameter " << name << ", using default bool.\n" << RESET;
    return false;
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    if (std::holds_alternative<uint32_t>(*result)) {
      return static_cast<uint8_t>(std::get<uint32_t>(*result));
    }
    std::cerr << RED << "Wrong type for parameter " << name << ", using default uint8_t.\n" <<
      RESET;
    return 0;
  } else if constexpr (std::is_same_v<T, uint16_t>) {
    if (std::holds_alternative<uint32_t>(*result)) {
      return static_cast<uint16_t>(std::get<uint32_t>(*result));
    }
    std::cerr << RED << "Wrong type for parameter " << name << ", using default uint16_t.\n" <<
      RESET;
    return 0;
  } else if constexpr (std::is_same_v<T, uint32_t>) {
    if (std::holds_alternative<uint32_t>(*result)) {
      return std::get<uint32_t>(*result);
    }
    std::cerr << RED << "Wrong type for parameter " << name << ", using default uint32_t.\n" <<
      RESET;
    return 0;
  } else {
    static_assert(sizeof(T) == 0, "Unsupported type");
  }
}

template<typename T> T SparkBase::GetPIDParam(Parameter baseParam, uint8_t slot, const char * name)
{
  if (slot >= 4) {
    throw std::out_of_range("Invalid slot number");
  }
  return GetParamAs<T>(static_cast<Parameter>(static_cast<int>(baseParam) + slot), name);
}

std::optional<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t, bool>> SparkBase::ReadFirmwareVersion()
{
  struct can_frame request = {};
  uint32_t requestarbId = CreateArbId(APICommand::FirmwareVersion);
  request.can_id = requestarbId | CAN_EFF_FLAG;
  request.can_dlc = 0;

  if (write(soc_, &request, sizeof(request)) < 0) {
    perror("Error sending firmware version request");
    return std::nullopt;
  }

  struct can_frame response = {};
  struct timeval tv = {0, READ_TIMEOUT_US};
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(soc_, &read_fds);

  int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret > 0) {
    ssize_t bytesRead = read(soc_, &response, sizeof(response));
    if (bytesRead >= 5) {
      uint32_t receivedarbId = response.can_id & CAN_EFF_MASK;
      if (receivedarbId == requestarbId) {
        uint8_t major = response.data[0];
        uint8_t minor = response.data[1];
        uint8_t patch = response.data[2];
        uint8_t build = response.data[3];
        bool isDebug = response.data[4] != 0;
        return std::make_tuple(major, minor, patch, build, isDebug);
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

  while (run_) {
    struct timeval tv = {0, READ_TIMEOUT_US};
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(soc_, &read_fds);

    int ret = select(soc_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0) {
      ssize_t bytesRead = read(soc_, &response, sizeof(response));
      if (bytesRead <= 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          // Try again
          break;
        }
        continue;
      }

      // Process the frame
      uint32_t receivedArbId = response.can_id & CAN_EFF_MASK;
      uint64_t rawValue = 0;
      for (int i = 0; i < response.can_dlc; ++i) {
        rawValue |= uint64_t(response.data[i]) << (8 * i);
      }
      auto now = std::chrono::steady_clock::now();

      std::lock_guard<std::mutex> lock(mutex_);

      if (receivedArbId == CreateArbId(APICommand::Period0)) {
        period0_.dutyCycle = int16_t(rawValue & 0xFFFF) / 32768.0f;
        period0_.faults = (rawValue >> 16) & 0xFFFF;
        period0_.stickyFaults = (rawValue >> 32) & 0xFFFF;
        period0_.isInverted = (rawValue >> 49) & 1;
        period0_.idleMode = (rawValue >> 57) & 1;
        period0_.isFollower = (rawValue >> 58) & 1;
        period0_.timestamp = now;

      } else if (receivedArbId == CreateArbId(APICommand::Period1)) {
        uint32_t velocity = rawValue & 0xFFFFFFFF;
        std::memcpy(&period1_.velocity, &velocity, 4);
        period1_.temperature = (rawValue >> 32) & 0xFF;
        period1_.voltage = ((rawValue >> 40) & 0xFFFF) / 128.0f;
        period1_.current = ((rawValue >> 48) & 0xFFF) / 32.0f;
        period1_.timestamp = now;
      } else if (receivedArbId == CreateArbId(APICommand::Period2)) {
        uint32_t position = rawValue & 0xFFFFFFFF;
        std::memcpy(&period2_.position, &position, 4);
        period2_.iAccum = float((rawValue >> 32) & 0xFFFFFFFF) / 1000.0f;
        period2_.timestamp = now;
      } else if (receivedArbId == CreateArbId(APICommand::Period3)) {
        uint8_t * intVal = reinterpret_cast<uint8_t *>(&rawValue);
        uint16_t voltage = intVal[0] | ((intVal[1] & 3) << 8);
        period3_.analogVoltage = float(voltage) / 256.0f;
        uint32_t velocity =
          ((intVal[1] >> 2) & 0x3F) | (uint32_t(intVal[2]) << 6) | (uint32_t(intVal[3]) << 14);
        period3_.analogVelocity = float(velocity) / 32768.0f;
        uint32_t position = (rawValue >> 32) & 0xFFFFFFFF;
        std::memcpy(&period3_.analogPosition, &position, 4);
        period3_.timestamp = now;
      } else if (receivedArbId == CreateArbId(APICommand::Period4)) {
        uint32_t velocity = rawValue & 0xFFFFFFFF;
        uint32_t position = (rawValue >> 32) & 0xFFFFFFFF;
        std::memcpy(&period4_.altEncoderVelocity, &velocity, 4);
        std::memcpy(&period4_.altEncoderPosition, &position, 4);
        period4_.timestamp = now;
      }
    }
  }
}

void SparkBase::Heartbeat()
{
  std::vector<uint8_t> data(8, 0xFF);
  SendCanFrame(APICommand::Heartbeat, data);
}

void SparkBase::BurnFlash()
{
  std::vector<uint8_t> data = {0xA3, 0x3A};
  SendCanFrame(APICommand::BurnFlash, data);
}

void SparkBase::FactoryDefaults()
{
  std::vector<uint8_t> data = {0x01};
  SendCanFrame(APICommand::FactoryDefaults, data);
}

void SparkBase::FactoryReset()
{
  std::vector<uint8_t> data = {0x01};
  SendCanFrame(APICommand::FactoryReset, data);
}

void SparkBase::Identify()
{
  std::vector<uint8_t> data(8, 0x00);
  SendCanFrame(APICommand::Identify, data);
}

void SparkBase::ResetFaults()
{
  std::vector<uint8_t> data(8, 0x00);
  SendCanFrame(APICommand::ClearFaults, data);
}

void SparkBase::ClearStickyFaults()
{
  std::vector<uint8_t> data(8, 0x00);
  SendCanFrame(APICommand::ClearFaults, data);
}

// Motor Control //
void SparkBase::SetSetpoint(float setpoint)
{
  SendControlMessage(APICommand::Setpoint, "Setpoint", setpoint);
}

void SparkBase::SetDutyCycle(float dutyCycle)
{
  SendControlMessage(APICommand::DutyCycle, "Duty Cycle", dutyCycle, -1.0f, 1.0f);
}

void SparkBase::SetVelocity(float velocity)
{
  SendControlMessage(APICommand::Velocity, "Velocity", velocity);
}

void SparkBase::SetSmartVelocity(float smartVelocity)
{
  SendControlMessage(APICommand::SmartVelocity, "Smart Velocity", smartVelocity);
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

void SparkBase::SetSmartMotion(float smartMotion)
{
  SendControlMessage(APICommand::SmartMotion, "Smart Motion", smartMotion);
}

// Status //

void SparkBase::SetPeriodicStatus0Period(uint16_t period)
{
  std::vector<uint8_t> data(2, 0x00);
  data[0] = static_cast<uint8_t>(period & 0xFF);
  data[1] = static_cast<uint8_t>((period >> 8) & 0xFF);
  SendCanFrame(APICommand::Period0, data);
}

void SparkBase::SetPeriodicStatus1Period(uint16_t period)
{
  std::vector<uint8_t> data(2, 0x00);
  data[0] = static_cast<uint8_t>(period & 0xFF);
  data[1] = static_cast<uint8_t>((period >> 8) & 0xFF);
  SendCanFrame(APICommand::Period1, data);
}

void SparkBase::SetPeriodicStatus2Period(uint16_t period)
{
  std::vector<uint8_t> data(2, 0x00);
  data[0] = static_cast<uint8_t>(period & 0xFF);
  data[1] = static_cast<uint8_t>((period >> 8) & 0xFF);
  SendCanFrame(APICommand::Period2, data);
}

void SparkBase::SetPeriodicStatus3Period(uint16_t period)
{
  std::vector<uint8_t> data(2, 0x00);
  data[0] = static_cast<uint8_t>(period & 0xFF);
  data[1] = static_cast<uint8_t>((period >> 8) & 0xFF);
  SendCanFrame(APICommand::Period3, data);
}

void SparkBase::SetPeriodicStatus4Period(uint16_t period)
{
  std::vector<uint8_t> data(2, 0x00);
  data[0] = static_cast<uint8_t>(period & 0xFF);
  data[1] = static_cast<uint8_t>((period >> 8) & 0xFF);
  SendCanFrame(APICommand::Period4, data);
}

// Period 0
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

// Period 1
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

// Period 2
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

// Period 3
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

// Period 4
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

// Parameter Setters //

// Basic //

void SparkBase::SetMotorType(MotorType type)
{
  SetParameter(
    Parameter::kMotorType, PARAM_TYPE_UINT, "Motor Type", static_cast<uint8_t>(type), 0, 1,
    "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless).");
}

void SparkBase::SetSensorType(SensorType type)
{
  SetParameter(
    Parameter::kSensorType, PARAM_TYPE_UINT, "Sensor Type", static_cast<uint8_t>(type), 0, 2,
    "Invalid sensor type. Must be 0 (No Sensor), 1 (Hall Sensor), or 2 (Encoder).");
}

void SparkBase::SetIdleMode(IdleMode mode)
{
  SetParameter(
    Parameter::kIdleMode, PARAM_TYPE_UINT, "Idle Mode", static_cast<uint8_t>(mode), 0, 1,
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

void SparkBase::SetCtrlType(CtrlType type)
{
  SetParameter(
    Parameter::kCtrlType, PARAM_TYPE_UINT, "Control Type", static_cast<uint8_t>(type), 0, 3,
    "Invalid control type. Must be 0 (Duty Cycle), 1 (Velocity), 2 (Voltage), or 3 (Position).");
}

void SparkBase::SetFeedbackSensorPID0(uint16_t sensor)
{
  SetParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, "Feedback Sensor PID0", sensor);
}

void SparkBase::SetClosedLoopVoltageMode(uint8_t mode)
{
  SetParameter(
    Parameter::kClosedLoopVoltageMode, PARAM_TYPE_UINT, "Closed Loop Voltage Mode", mode, 0, 2,
    "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control Loop Voltage Output Mode) or 2 "
    "(Voltage Compensation Mode).");
}

void SparkBase::SetCompensatedNominalVoltage(float voltage)
{
  SetParameter(
    Parameter::kCompensatedNominalVoltage, PARAM_TYPE_FLOAT,
    "Compensated Nominal Voltage", voltage);
}

void SparkBase::SetPositionPIDWrapEnable(bool enable)
{
  SetParameter(
    Parameter::kPositionPIDWrapEnable, PARAM_TYPE_BOOL, "Position PID Wrap Enable",
    enable);
}

void SparkBase::SetPositionPIDMinInput(float minInput)
{
  SetParameter(
    Parameter::kPositionPIDMinInput, PARAM_TYPE_FLOAT, "Position PID Min Input",
    minInput);
}

void SparkBase::SetPositionPIDMaxInput(float maxInput)
{
  SetParameter(
    Parameter::kPositionPIDMaxInput, PARAM_TYPE_FLOAT, "Position PID Max Input",
    maxInput);
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
  SetParameter(
    Parameter::kSmartCurrentStallLimit, PARAM_TYPE_UINT, "Smart Current Stall Limit",
    limit);
}

void SparkBase::SetSmartCurrentFreeLimit(uint16_t limit)
{
  SetParameter(
    Parameter::kSmartCurrentFreeLimit, PARAM_TYPE_UINT, "Smart Current Free Limit",
    limit);
}

void SparkBase::SetSmartCurrentConfig(uint16_t config)
{
  SetParameter(Parameter::kSmartCurrentConfig, PARAM_TYPE_UINT, "Smart Current Config", config);
}

// PIDF //

void SparkBase::SetP(uint8_t slot, float p)
{
  static const std::array<Parameter,
    4> params = {Parameter::kP_0, Parameter::kP_1, Parameter::kP_2, Parameter::kP_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "P", p);
}

void SparkBase::SetI(uint8_t slot, float i)
{
  static const std::array<Parameter,
    4> params = {Parameter::kI_0, Parameter::kI_1, Parameter::kI_2, Parameter::kI_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "I", i);
}

void SparkBase::SetD(uint8_t slot, float d)
{
  static const std::array<Parameter,
    4> params = {Parameter::kD_0, Parameter::kD_1, Parameter::kD_2, Parameter::kD_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "D", d);
}

void SparkBase::SetF(uint8_t slot, float f)
{
  static const std::array<Parameter,
    4> params = {Parameter::kF_0, Parameter::kF_1, Parameter::kF_2, Parameter::kF_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "F", f);
}

void SparkBase::SetIZone(uint8_t slot, float iZone)
{
  static const std::array<Parameter,
    4> params = {Parameter::kIZone_0, Parameter::kIZone_1, Parameter::kIZone_2,
    Parameter::kIZone_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "IZone", iZone);
}

void SparkBase::SetDFilter(uint8_t slot, float dFilter)
{
  static const std::array<Parameter,
    4> params = {Parameter::kDFilter_0, Parameter::kDFilter_1, Parameter::kDFilter_2,
    Parameter::kDFilter_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "DFilter", dFilter);
}

void SparkBase::SetOutputMin(uint8_t slot, float min)
{
  static const std::array<Parameter, 4> params = {Parameter::kOutputMin_0, Parameter::kOutputMin_1,
    Parameter::kOutputMin_2, Parameter::kOutputMin_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Output Min", min);
}

void SparkBase::SetOutputMax(uint8_t slot, float max)
{
  static const std::array<Parameter, 4> params = {Parameter::kOutputMax_0, Parameter::kOutputMax_1,
    Parameter::kOutputMax_2, Parameter::kOutputMax_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Output Max", max);
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
  SetParameter(
    Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, "Limit Switch Forward Polarity",
    polarity);
}

void SparkBase::SetLimitSwitchRevPolarity(bool polarity)
{
  SetParameter(
    Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, "Limit Switch Reverse Polarity",
    polarity);
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
  SetParameter(
    Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, "Encoder Counts Per Revolution",
    counts);
}

void SparkBase::SetEncoderAverageDepth(uint8_t depth)
{
  SetParameter(
    Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, "Encoder Average Depth", depth, 1,
    64);
}

void SparkBase::SetEncoderSampleDelta(uint8_t delta)
{
  SetParameter(
    Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, "Encoder Sample Delta", delta, 1,
    255);
}

void SparkBase::SetEncoderInverted(bool inverted)
{
  SetParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, "Encoder Inverted", inverted);
}

void SparkBase::SetPositionConversionFactor(float factor)
{
  SetParameter(
    Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, "Position Conversion Factor",
    factor);
}

void SparkBase::SetVelocityConversionFactor(float factor)
{
  SetParameter(
    Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, "Velocity Conversion Factor",
    factor);
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
  SetParameter(
    Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, "Hall Sensor Average Depth",
    depth);
}

// Smart Motion //

void SparkBase::SetSmartMotionMaxVelocity(uint8_t slot, float maxVel)
{
  static const std::array<Parameter, 4> params = {
    Parameter::kSmartMotionMaxVelocity_0, Parameter::kSmartMotionMaxVelocity_1,
    Parameter::kSmartMotionMaxVelocity_2, Parameter::kSmartMotionMaxVelocity_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Velocity", maxVel);
}

void SparkBase::SetSmartMotionMaxAccel(uint8_t slot, float maxAccel)
{
  static const std::array<Parameter, 4> params = {
    Parameter::kSmartMotionMaxAccel_0, Parameter::kSmartMotionMaxAccel_1,
    Parameter::kSmartMotionMaxAccel_2,
    Parameter::kSmartMotionMaxAccel_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Accel", maxAccel);
}

void SparkBase::SetSmartMotionMinVelOutput(uint8_t slot, float minVel)
{
  static const std::array<Parameter, 4> params = {
    Parameter::kSmartMotionMinVelOutput_0, Parameter::kSmartMotionMinVelOutput_1,
    Parameter::kSmartMotionMinVelOutput_2, Parameter::kSmartMotionMinVelOutput_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Min Vel Output", minVel);
}

void SparkBase::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error)
{
  static const std::array<Parameter, 4> params = {
    Parameter::kSmartMotionAllowedClosedLoopError_0,
    Parameter::kSmartMotionAllowedClosedLoopError_1,
    Parameter::kSmartMotionAllowedClosedLoopError_2,
    Parameter::kSmartMotionAllowedClosedLoopError_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Allowed Close Loop Error", error);
}

void SparkBase::SetSmartMotionAccelStrategy(uint8_t slot, float strategy)
{
  static const std::array<Parameter, 4> params = {
    Parameter::kSmartMotionAccelStrategy_0, Parameter::kSmartMotionAccelStrategy_1,
    Parameter::kSmartMotionAccelStrategy_2, Parameter::kSmartMotionAccelStrategy_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Accel Strategy", strategy);
}

void SparkBase::SetIMaxAccum(uint8_t slot, float maxAccum)
{
  static const std::array<Parameter, 4> params = {Parameter::kIMaxAccum_0, Parameter::kIMaxAccum_1,
    Parameter::kIMaxAccum_2, Parameter::kIMaxAccum_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "IMaxAccum", maxAccum);
}

void SparkBase::SetSlot3Placeholder1(uint8_t slot, float value)
{
  static const std::array<Parameter,
    4> params = {Parameter::kSlot3Placeholder1_0, Parameter::kSlot3Placeholder1_1,
    Parameter::kSlot3Placeholder1_2, Parameter::kSlot3Placeholder1_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 1", value);
}

void SparkBase::SetSlot3Placeholder2(uint8_t slot, float value)
{
  static const std::array<Parameter,
    4> params = {Parameter::kSlot3Placeholder2_0, Parameter::kSlot3Placeholder2_1,
    Parameter::kSlot3Placeholder2_2, Parameter::kSlot3Placeholder2_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 2", value);
}

void SparkBase::SetSlot3Placeholder3(uint8_t slot, float value)
{
  static const std::array<Parameter,
    4> params = {Parameter::kSlot3Placeholder3_0, Parameter::kSlot3Placeholder3_1,
    Parameter::kSlot3Placeholder3_2, Parameter::kSlot3Placeholder3_3};

  if (slot >= params.size()) {
    throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
  }

  SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 3", value);
}

// Analog Sensor //

void SparkBase::SetAnalogPositionConversion(float factor)
{
  SetParameter(
    Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, "Analog Position Conversion",
    factor);
}

void SparkBase::SetAnalogVelocityConversion(float factor)
{
  SetParameter(
    Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, "Analog Velocity Conversion",
    factor);
}

void SparkBase::SetAnalogAverageDepth(uint16_t depth)
{
  SetParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, "Analog Average Depth", depth);
}

void SparkBase::SetAnalogSensorMode(uint8_t mode)
{
  SetParameter(
    Parameter::kAnalogSensorMode, PARAM_TYPE_UINT, "Analog Sensor Mode", mode, 0, 1,
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
  SetParameter(
    Parameter::kDataPortConfig, PARAM_TYPE_UINT, "Data Port Config", config, 0, 1,
    "Invalid data port config. Must be 0 (Default) or 1 (Alternate Encoder Mode).");
}

void SparkBase::SetAltEncoderCountsPerRev(uint16_t counts)
{
  SetParameter(
    Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, "Alternate Encoder Counts Per Revolution",
    counts);
}

void SparkBase::SetAltEncoderAverageDepth(uint8_t depth)
{
  SetParameter(
    Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT,
    "Alternate Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetAltEncoderSampleDelta(uint8_t delta)
{
  SetParameter(
    Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, "Alternate Encoder Sample Delta",
    delta, 1, 255);
}

void SparkBase::SetAltEncoderInverted(bool inverted)
{
  SetParameter(
    Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, "Alternate Encoder Inverted",
    inverted);
}

void SparkBase::SetAltEncoderPositionFactor(float factor)
{
  SetParameter(
    Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT,
    "Alternate Encoder Position Factor", factor);
}

void SparkBase::SetAltEncoderVelocityFactor(float factor)
{
  SetParameter(
    Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT,
    "Alternate Encoder Velocity Factor", factor);
}

// Duty Cycle Absolute Encoder //

void SparkBase::SetDutyCyclePositionFactor(float factor)
{
  SetParameter(
    Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, "Duty Cycle Position Factor",
    factor);
}

void SparkBase::SetDutyCycleVelocityFactor(float factor)
{
  SetParameter(
    Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, "Duty Cycle Velocity Factor",
    factor);
}

void SparkBase::SetDutyCycleInverted(bool inverted)
{
  SetParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, "Duty Cycle Inverted", inverted);
}

void SparkBase::SetDutyCycleAverageDepth(uint8_t depth)
{
  SetParameter(
    Parameter::kDutyCycleAverageDepth, PARAM_TYPE_UINT, "Duty Cycle Average Depth", depth, 0, 7,
    "Invalid average depth. Must be 0 (1 bit), 2 (2 bits), 3 (4 bits), 4 (8 bits), 5 (16 bits), 6 (32 "
    "bits), or 7 (64 bits).");
}

void SparkBase::SetDutyCyclePrescalar(uint8_t prescalar)
{
  SetParameter(
    Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, "Duty Cycle Prescalar", prescalar,
    0, 71);
}

void SparkBase::SetDutyCycleZeroOffset(float offset)
{
  SetParameter(
    Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, "Duty Cycle Zero Offset", offset,
    0.0f, 1.0f);
}

// Parameter Getters //

// Basic //

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

// Advanced //

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

// Closed Loop //

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

// Brushless //

uint16_t SparkBase::GetPolePairs()
{
  return GetParamAs<uint16_t>(Parameter::kPolePairs, "PolePairs");
}

// Current Limit //

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

// PIDF //

float SparkBase::GetP(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kP_0, slot, "P");
}

float SparkBase::GetI(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kI_0, slot, "I");
}

float SparkBase::GetD(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kD_0, slot, "D");
}

float SparkBase::GetF(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kF_0, slot, "F");
}

float SparkBase::GetIZone(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kIZone_0, slot, "IZone");
}

float SparkBase::GetDFilter(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kDFilter_0, slot, "DFilter");
}

float SparkBase::GetOutputMin(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kOutputMin_0, slot, "OutputMin");
}

float SparkBase::GetOutputMax(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kOutputMax_0, slot, "OutputMax");
}

// Limits //

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

// Follower //

uint32_t SparkBase::GetFollowerID()
{
  return GetParamAs<uint32_t>(Parameter::kFollowerID, "FollowerID");
}

uint32_t SparkBase::GetFollowerConfig()
{
  return GetParamAs<uint32_t>(Parameter::kFollowerConfig, "FollowerConfig");
}

// Encoder Port //

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

// Smart Motion //

float SparkBase::GetSmartMotionMaxVelocity(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSmartMotionMaxVelocity_0, slot, "SmartMotionMaxVelocity");
}

float SparkBase::GetSmartMotionMaxAccel(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSmartMotionMaxAccel_0, slot, "SmartMotionMaxAccel");
}

float SparkBase::GetSmartMotionMinVelOutput(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSmartMotionMinVelOutput_0, slot, "SmartMotionMinVelOutput");
}

float SparkBase::GetSmartMotionAllowedClosedLoopError(uint8_t slot)
{
  return GetPIDParam<float>(
    Parameter::kSmartMotionAllowedClosedLoopError_0, slot,
    "SmartMotionAllowedClosedLoopError");
}

float SparkBase::GetSmartMotionAccelStrategy(uint8_t slot)
{
  return GetPIDParam<float>(
    Parameter::kSmartMotionAccelStrategy_0, slot,
    "SmartMotionAccelStrategy");
}

float SparkBase::GetIMaxAccum(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kIMaxAccum_0, slot, "IMaxAccum");
}

float SparkBase::GetSlot3Placeholder1(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSlot3Placeholder1_0, slot, "Slot3Placeholder1");
}

float SparkBase::GetSlot3Placeholder2(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSlot3Placeholder2_0, slot, "Slot3Placeholder2");
}

float SparkBase::GetSlot3Placeholder3(uint8_t slot)
{
  return GetPIDParam<float>(Parameter::kSlot3Placeholder3_0, slot, "Slot3Placeholder3");
}

// Analog Sensor //

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

// Alternate Encoder //

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

// Duty Cycle Absolute Encoder //

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
