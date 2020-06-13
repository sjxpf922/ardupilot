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
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>

#define DEG_TO_RAD_MTI        0.0174532925f


class AP_MTi_G {
public:
    AP_MTi_G();

    /* Do not allow copies */
    AP_MTi_G(const AP_MTi_G &other) = delete;
    AP_MTi_G &operator=(const AP_MTi_G&) = delete;

    // init - perform required initialisation
    bool init();
    bool Read_Mti_AHRS(void);
    void Mti_ReceiveData(uint8_t temp);
    void Mti_Parsing(uint8_t Id,uint8_t * data,uint8_t Len);
    int wrap_360_cd_yaw(int yaw_change);
private:

    static const uint8_t UART_PREAMBLE1 = 0xaa;  //数据帧头 1
    static const uint8_t UART_PREAMBLE2 = 0x44;    //2
    static const uint8_t DataId1 = 0x50;  //速度数据ID
    static const uint8_t DataId2 = 0x55;  //高度数据ID

    uint8_t mti_state;             //状态标志位
    uint8_t checksum;              //校验和
    uint8_t MID;                   //Message identifier
    uint8_t MessLen;              //有效数据长度
    int readnum ;                 //所有有效数据（从DATA开始）的个数（字节）
    uint8_t p_data;               //数据id的个数0-254;
    uint8_t buff[255];              //
    uint8_t DataId ;              //数据ID
    uint8_t Data_Len;             //每种类型的数据长度
    uint8_t  mti_register;        //用于标注信息类型

    enum{
            Attitude_Angle = 0,  //姿态角
            Accel             ,  //加速度
            Turn_Rate         ,  //转速
            Velocity          ,  //速度
            Altitude          ,  //高度
            Lng_Lat           ,  //经纬高
            Air_Pressure      ,  //气压
            Tempeature        ,  //温度

    };
    struct  {
            Vector3f  MTI_acce;  //m/s^2  NED body
            Vector3f  MTI_Gyr;   //rad/s
            Vector3f  MTI_attitude;//rad
            Vector3f  MTI_Velocity;//m/s  NED
            Vector3f  MTI_magn;
            int32_t   MTI_Lat;//*10e7
            int32_t   MTI_Lon;//*10e7
            double    MTI_Alt;//cm
            double    MTI_pressure;
            float     MTI_temp;
        }  MTI_ins;



    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter


};
