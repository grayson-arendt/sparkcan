#include <chrono>
#include <iomanip>
#include <iostream>

#include "SparkMax.hpp"

/*
This has been tested with the SPARK MAX while connected to a NEO V1.1.
*/

int main()
{
  try {
    SparkMax motor("can0", 1);

    // PID configuration (Tune as needed, values will be very small)
    motor.SetP(0, 0.00005);
    motor.SetI(0, 0.0);
    motor.SetD(0, 0.0);
    motor.SetF(0, 0.00025);
    motor.BurnFlash();

    float targetVelocityRPM = 200.0f; // Set your desired RPM here

    auto startTime = std::chrono::high_resolution_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - startTime)
      .count() < 10)
    {

      motor.Heartbeat();
      motor.SetVelocity(targetVelocityRPM);

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Target: " << targetVelocityRPM << " RPM | ";
      std::cout << "Actual: " << motor.GetVelocity() << " RPM | ";
      std::cout << "Duty: " << motor.GetDutyCycle() * 100.0f << "% | ";
      std::cout << "Current: " << motor.GetCurrent() << " A\n";
    }

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
