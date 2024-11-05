#include "SparkMax.hpp"
#include "SparkFlex.hpp"
#include <iostream>

/*
This has been tested with the SPARK MAX while connected to an AndyMark 775 RedLine motor and
with a Spark Flex connected to a NEO Vortex motor.
*/

int main()
{
    try
    {
        // Initialize SparkMax object with CAN interface and CAN ID
        SparkMax motor("can0", 47);
        SparkFlex motor2("can0", 15);

        // Motor 1
        motor.SetIdleMode(1);  // Brake
        motor.SetMotorType(0); // Brushed
        motor.SetInverted(true);
        motor.BurnFlash();

        // Motor 2
        motor2.SetIdleMode(1); // Brake
        motor2.SetRampRate(0.1);
        motor2.SetInverted(false);
        motor2.SetMotorKv(565);
        motor2.SetEncoderCountsPerRev(7168);
        motor2.SetSensorType(1);
        motor2.SetSmartCurrentFreeLimit(20.0);
        motor2.SetSmartCurrentStallLimit(20.0);
        motor2.BurnFlash();
   
        // Loop for 10 seconds
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count() < 10)
        {
            motor.Heartbeat();
            motor2.Heartbeat();

            motor.SetDutyCycle(0.05);
            motor2.SetDutyCycle(0.1);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
