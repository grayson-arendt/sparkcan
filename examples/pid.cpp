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
    // Initialize SparkMax object with CAN interface and CAN ID
    SparkMax motor("can0", 1);

    // PID configuration (Tune as needed, values will be very small)
    motor.SetP(0, 0.00005);
    motor.SetI(0, 0.0);
    motor.SetD(0, 0.0);
    motor.SetF(0, 0.00025);
    motor.BurnFlash();

    float targetVelocity = 200.0; // Set your desired RPM here

    // Loop for 10 seconds
    auto start = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() -
        start)
      .count() < 10)
    {
      // Enable and set velocity
      motor.Heartbeat();
      motor.SetVelocity(targetVelocity);

      // Display motor status
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\r";
      std::cout << "Target: " << targetVelocity << " RPM | ";
      std::cout << "Actual: " << motor.GetVelocity() << " RPM";
      std::cout.flush();
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
