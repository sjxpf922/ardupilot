#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_TestUart {
public:
    AP_TestUart();
    /* Do not allow copies */
    AP_TestUart(const AP_TestUart &other) = delete;
    AP_TestUart &operator=(const AP_TestUart&) = delete;

    // init - perform required initialisation
    bool init();
    float ChangeSpeed(void);
    float airspeed;
private:
  //by sjx to charge airspeed
    union PACKED {
        float speed;
        uint8_t buff[4];
    }chartofloat;

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
};
