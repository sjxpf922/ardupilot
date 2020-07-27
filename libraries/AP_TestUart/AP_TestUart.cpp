
/* 
  TestUart _ 测试pixhawk 串口与电脑通讯  _by TongXueShi
*/
#define AP_SERIALMANAGER_TestUart_BAUD        115200
#define AP_SERIALMANAGER_TestUart_BUFSIZE_RX      64
#define AP_SERIALMANAGER_TestUart_BUFSIZE_TX      64

#include <AP_TestUart/AP_TestUart.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;



//construct
AP_TestUart::AP_TestUart(void)
{
    _protocol = AP_SerialManager::SerialProtocol_None; //默认初始化为没有
    _port = NULL;                                      //默认初始化为空
    airspeed = 0.0;

}

 /* init - perform required initialisation*/


bool AP_TestUart::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TestUart, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_TestUart; // FrSky D protocol (D-receivers)
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //Initialize the uart
        _port->begin(AP_SERIALMANAGER_TestUart_BAUD,AP_SERIALMANAGER_TestUart_BUFSIZE_RX, AP_SERIALMANAGER_TestUart_BUFSIZE_TX);
        return true;
    }
    return false;

}

float AP_TestUart :: ChangeSpeed(void)
{

    if( _port ==NULL)
            return false;
   // int16_t numc = _port->available();
     int16_t i = 0;
    for(i = 0; i < 4; i ++)
    {
        chartofloat.buff[i]=_port -> read();
        _buff[i] = chartofloat.buff[i];
    }
    return airspeed = chartofloat.speed;
}


