////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021 Mateusz Malinowski
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <unistd.h>

#include <icm20948.h>

/** Scale to convert angles from radians to degrees. */
static const float RAD_TO_DEG = 180.0f / 3.14159f;

/**
 * Prints help information about this application.
 *  @param name the name of the executable.
 */
void printHelp(const char* name)
{
    printf("Usage: %s [options] \n", name);
    printf("    -h, --help  -> prints this information \n");
    printf("    -l, --loop  -> sets the maximum number of loop iterations \n");
    printf("    -c, --calib -> if set to one triggers gyro calibration \n");
    printf("\nExample: %s -l 2000 -c 1 \n", name);
}

/**
 *  @return the time of the system in seconds.
 */
double getTime()
{
    struct timespec timeStruct;
    clock_gettime(CLOCK_REALTIME, &timeStruct);
    return static_cast<double>(timeStruct.tv_sec) + static_cast<double>(timeStruct.tv_nsec / 1000000) / 1000;
}

/**
 * Clears the terminal screen and moves cursor to top-left
 */
void clearScreen()
{
    printf("\033[2J\033[H");
    fflush(stdout);
}

/**
 * Creates a high-rate configuration for maximum sensor performance
 * @return ICM20948::Config configured for high sample rates
 */
ICM20948::Config createHighRateConfig()
{
    ICM20948::Config config;
    
    // Set frame rate to 200Hz
    config.mFramerate = 200.0f; // Hardcoded to 200Hz
    
    // Configure gyroscope for high rate
    config.mGyro.mSampleRateDivisor = 0; // Minimum divisor for max rate
    config.mGyro.mDLPFBandwidth = ICM20948::GYRO_DLPF_BANDWIDTH_361HZ; // Highest bandwidth
    config.mGyro.mAveraging = ICM20948::GYRO_AVERAGING_1X; // No averaging for max speed
    
    // Configure accelerometer for high rate  
    config.mAcc.mSampleRateDivisor = 0; // Minimum divisor for max rate
    config.mAcc.mDLPFBandwidth = ICM20948::ACC_DLPF_BANDWIDTH_473HZ; // Highest bandwidth
    config.mAcc.mAveraging = ICM20948::ACC_AVERAGING_NONE; // No averaging for max speed
    
    // Disable magnetometer for higher rates (it's slower)
    config.mMagEnabled = false;
    
    return config;
}

int main(int argc, char** argv)
{
    /** Current time in seconds to calculate time needed to sleep for the main loop. */
    double currentTime;
    /** Previous time for rate calculation */
    double previousTime = 0;
    /** Loop iterator */
    ulong i;
    /** The maximum number of loops. */
    ulong max = 2000;
    /** Time to sleep in milliseconds for the main loop. */
    int sleepTime;
    /** Flag to indicate of gyro calibration should be performed. */
    bool calib = true;
    /** IMU implementation. */
    ICM20948 imu;
    /** Pointer for IMU data.  */
    const IMUData* data;

    /* Parse any inputs that may have been provided. */
    for (i = 1; i < static_cast<ulong>(argc); ++i)
    {
        if ((0 == strcmp(argv[i], "--loop")) || (0 == strcmp(argv[i], "-l")))
        {
            max = static_cast<ulong>(atol(argv[i + 1]));
        }
        else if ((0 == strcmp(argv[i], "--calib")) || (0 == strcmp(argv[i], "-c")))
        {
            calib = static_cast<bool>(atoi(argv[i + 1]));
        }
        else if ((0 == strcmp(argv[i], "--help")) || (0 == strcmp(argv[i], "-h")))
        {
            printHelp(argv[0]);
            return 0;
        }
        else
        {
            /* nothing to do in here */
        }
    }

    // Use high-rate configuration
    ICM20948::Config highRateConfig = createHighRateConfig();
    
    if (imu.initialise(highRateConfig))
    {
        puts("ICM-20948 detected successfully");
        if (calib)
        {
            puts("Performing gyroscope calibration");
            imu.calibrateGyro();
        }
        
        // Clear screen once before starting the loop
        clearScreen();
        
        for (i = 0; i < max; ++i)
        {
            currentTime = getTime();
            data = &imu.imuDataGet();
            
            // Clear screen and move cursor to top
            clearScreen();
            
            // Calculate write rate
            double writeRate = 0.0;
            if (previousTime != 0)
            {
                double deltaTime = currentTime - previousTime;
                writeRate = (deltaTime > 0) ? (1.0 / deltaTime) : 0.0;
            }
            
            printf("=== ICM-20948 Real-time Data ===\n");
            printf("Counter: %lu / %lu\n", i + 1, max);
            printf("Write Rate: %.1f Hz\n", writeRate);
            printf("Expected Rate: %.1f Hz\n\n", imu.getConfig().mFramerate);
            
            printf("Angles (Roll/Pitch/Yaw):\n");
            printf("  Roll:  %12.6f°\n", data->mAngles[0]);
            printf("  Pitch: %12.6f°\n", data->mAngles[1]);
            printf("  Yaw:   %12.6f°\n\n", data->mAngles[2]);
            
            printf("Acceleration (g):\n");
            printf("  X: %12.8f\n", data->mAcc[0]);
            printf("  Y: %12.8f\n", data->mAcc[1]);
            printf("  Z: %12.8f\n\n", data->mAcc[2]);
            
            printf("Gyroscope (dps):\n");
            printf("  X: %12.8f\n", data->mGyro[0] * RAD_TO_DEG);
            printf("  Y: %12.8f\n", data->mGyro[1] * RAD_TO_DEG);
            printf("  Z: %12.8f\n\n", data->mGyro[2] * RAD_TO_DEG);
            
            // Uncomment these if you want magnetometer and temperature data
            // printf("Magnetic (uT):\n");
            // printf("  X: %12.6f\n", data->mMag[0]);
            // printf("  Y: %12.6f\n", data->mMag[1]);
            // printf("  Z: %12.6f\n\n", data->mMag[2]);
            // printf("Temperature: %.4f°C\n", data->mTemp);
            
            printf("Press Ctrl+C to stop...");
            fflush(stdout);
            
            previousTime = currentTime;
            
            /* For higher rates, use a more aggressive sleep calculation or remove sleep entirely */
            sleepTime = static_cast<int>(((1.0f / imu.getConfig().mFramerate) - (getTime() - currentTime)) * 1000000.0);
            /* Only sleep if processing was faster than target rate and sleep time is reasonable */
            if (sleepTime > 0 && sleepTime < 50000) // Max 50ms sleep
            {
                usleep(static_cast<uint>(sleepTime));
            }
            // For maximum rate, comment out the sleep entirely:
            // No sleep = maximum possible rate
        }
    }
    else
    {
        puts("fFailed to detect ICM-20948");
    }

    puts("finished");
    return 0;
}
