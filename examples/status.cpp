#include "SparkFlex.hpp"
#include <iomanip>
#include <iostream>

/*
This has been tested with the SPARK Flex while connected to a NEO Vortex.
*/

int main()
{
    try
    {
        // Initialize SparkFlex object with CAN interface and CAN ID
        SparkFlex motor("can0", 15);
        motor.SetIdleMode(1); // Brake
        motor.SetRampRate(0.1);
        motor.SetInverted(false);
        motor.SetMotorKv(565);
        motor.SetEncoderCountsPerRev(7168);
        motor.SetSensorType(1);
        motor.SetSmartCurrentFreeLimit(20.0);
        motor.SetSmartCurrentStallLimit(20.0);
        motor.BurnFlash();

        // Loop for 10 seconds
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                   .count() < 10)
        {
            motor.Heartbeat();
            motor.SetDutyCycle(0.2);

            std::cout << std::fixed << std::setprecision(2);

            std::cout << "Duty Cycle: " << motor.GetDutyCycle() << "" << std::endl;
            std::cout << "Velocity: " << motor.GetVelocity() << " RPM" << std::endl;
            std::cout << "Temperature: " << motor.GetTemperature() << " °C" << std::endl;
            std::cout << "Voltage: " << motor.GetVoltage() << " V" << std::endl;
            std::cout << "Current: " << motor.GetCurrent() << " A" << std::endl;
            std::cout << "Position: " << motor.GetPosition() << " ticks" << std::endl;
            std::cout << "Analog Voltage: " << motor.GetAnalogVoltage() << " V" << std::endl;
            std::cout << "Analog Velocity: " << motor.GetAnalogVelocity() << " RPM" << std::endl;
            std::cout << "Analog Position: " << motor.GetAnalogPosition() << " ticks" << std::endl;
            std::cout << "Alt Encoder Velocity: " << motor.GetAlternateEncoderVelocity() << " RPM" << std::endl;
            std::cout << "Alt Encoder Position: " << motor.GetAlternateEncoderPosition() << " ticks" << std::endl;
            std::cout << "Faults: " << motor.GetFaults() << std::endl;
            std::cout << "Sticky Faults: " << motor.GetStickyFaults() << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
