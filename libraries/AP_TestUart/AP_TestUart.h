/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
    float loop();
    bool senddata(int buff);
    float ChangeSpeed(void);
    float Asc_to_f(volatile unsigned char *str);
    bool  read(void);
    float airspeed;
    uint8_t _buff[4];
    void Serialdata_parsing(uint8_t temp);
    bool Read_Serial(void);
    void process_message();
    float Speed;
    float Height;
private:

    static const uint8_t UART_PREAMBLE1 = 0xaa;  //数据帧头 1
    static const uint8_t UART_PREAMBLE2 = 0x44;    //2
    static const uint8_t DataId1 = 0x50;  //速度数据ID
    static const uint8_t DataId2 = 0x55;  //高度数据ID

    struct PACKED Serial5_header
        {
            uint8_t headerlength;
            // 4
            uint16_t messageid;
            // 6
            uint8_t datalength;
            uint32_t crc;
        };
    struct PACKED parsingdata
        {

            float height;
            float velocity;

         };

    union PACKED msgbuffer {
        parsingdata _parsingdata;  //用于取高度相关的值
        uint8_t bytes[256];  //存有效数据的值
        };

    union PACKED msgheader {
        Serial5_header _header;  //用他的元素来取数据
            uint8_t data[28];          //用他来存帧头的数据
        };

    union PACKED msgcrc {
           uint32_t _crc;  //用他的元素来取数据
            uint8_t data[4];          //用他来存帧头的数据
           };
    struct PACKED HEAR{
        enum
            {
                PREAMBLE1 = 0,
                PREAMBLE2,
                HEADERLENGTH,
                DATA,
                DATALENGTH,
                _CRC
            } state;
              msgbuffer valid_data;//
              uint32_t crc;
              msgheader header;
              uint16_t read;
              msgcrc _msgcrs;

    } Serial5_msg;

  //by sjx to charge airspeed
    union PACKED {
        float speed;
        uint8_t buff[4];

    }chartofloat;

   /* union PACKED {
         uint16_t messageid; ;  //用他的元素来取数据
         uint8_t data[28];          //用他来存帧头的数据
    } header;
*/
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter


};
