#include "SparkMax.hpp"
#include <iostream>

/*
This has been tested with the SPARK MAX while connected to an AndyMark 775 RedLine motor.
*/

int main()
{
    try
    {
        // Initialize SparkMax object with CAN interface and CAN ID
        SparkMax motor("can0", 47);

        motor.SetIdleMode(1);  // Brake
        motor.SetMotorType(0); // Brushed
        motor.SetInverted(true);
        motor.BurnFlash();

        // Loop for 5 seconds
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count() < 5)
        {
            motor.Heartbeat();
            motor.SetAppliedOutput(0.2);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
