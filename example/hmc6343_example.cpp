/**
 * @file hmc6343_example.cpp
 * @author duckstarr
 * @brief A C++ program used to interface with the HMC6343 module.
 * 
 */

#include <hmc6343.hpp>
#include <iostream>

#include <signal.h>
#include <cstring>
#include <memory>

using namespace sensor;

// Signal handler callback function
volatile sig_atomic_t done = 0;
void sig_handler(int signum) 
{
    done = 1;
}

int main(int argc, char** argv)
{
  // Set up signal handler
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = sig_handler;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  // Start HMC6343.
  std::string device = "/dev/i2c-3";
  auto hmc6343_obj = std::unique_ptr<hmc6343>(new hmc6343(device.c_str()));

  // Set filter.
  hmc6343_obj->setFilter();

  while(!done)
  {
    printf("---\n");

    hmc6343_obj->readHeading();
    printf("Heading: %.2f\tPitch: %.2f\tRoll: %.2f\n", hmc6343_obj->heading / 10.0, hmc6343_obj->pitch / 10.0, hmc6343_obj->roll / 10.0);

    hmc6343_obj->readAccel();
    printf("Accel_X: %.2f\tAccel_Y: %.2f\tAccel_Z: %.2f\n", hmc6343_obj->accelX / 1024.0, hmc6343_obj->accelY / 1024.0, hmc6343_obj->accelZ / 1024.0);

    hmc6343_obj->readMag();
    printf("Mag_X: %.2f\tMag_Y: %.2f\tMag_Z: %.2f\n", hmc6343_obj->magX / 10.0, hmc6343_obj->magY / 10.0, hmc6343_obj->magZ / 10.0);

    hmc6343_obj->readTilt();
    printf("Pitch: %.2f\tRoll: %.2f\tTemp: %.2f\n", hmc6343_obj->pitch / 10.0, hmc6343_obj->roll / 10.0, hmc6343_obj->temperature / 10.0);

    printf("---\n");

    /**
     * @brief Output
        Heading: 10.00  Pitch: -2.70    Roll: -0.30
        Accel_X: 0.05   Accel_Y: 0.01   Accel_Z: -1.01
        Mag_X: 122.80   Mag_Y: 21.30    Mag_Z: -196.30
        Pitch: -2.70    Roll: -0.30     Temp: 21.70
      */
    fflush(stdout);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return EXIT_SUCCESS;
}
