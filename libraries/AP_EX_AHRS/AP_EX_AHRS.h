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
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include "AP_Common/Location.h"


class AP_EX_AHRS {
public:
    AP_EX_AHRS();

    /* Do not allow copies */
    AP_EX_AHRS(const AP_EX_AHRS &other) = delete;
    AP_EX_AHRS &operator=(const AP_EX_AHRS&) = delete;

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

    bool Read_Ex_AHRS(void);
    void Ex_ReceiveData(uint8_t temp);
    void Data_Parsing(uint8_t * data);
    void Data_Push(void);
    void printf_serial5(void);
    void set_ahrs_gyr(Vector3f gyr){_Ex_AHRS_Ins._AHRS_Gyr = gyr;}
    Vector3f get_ahrs_gyr(void)const{return _Ex_AHRS_Ins._AHRS_Gyr;}

    void set_ahrs_acc(Vector3f acc){_Ex_AHRS_Ins._AHRS_Acc = acc;}
    Vector3f get_ahrs_acc(void)const{return _Ex_AHRS_Ins._AHRS_Acc;}

    void set_ahrs_eulers(Vector3f eulers){_Ex_AHRS_Ins._AHRS_Eulers = eulers;}
    Vector3f get_ahrs_eulers(void)const{return _Ex_AHRS_Ins._AHRS_Eulers;}

    void set_ahrs_velocity(Vector3f vel){_Ex_AHRS_Ins._AHRS_Velocity = vel;}
    Vector3f get_ahrs_velocity(void)const{return _Ex_AHRS_Ins._AHRS_Velocity;}

    void set_ahrs_location(int32_t lat , int32_t lon, float alt ){_Ex_AHRS_Ins._AHRS_Lat= lat;_Ex_AHRS_Ins._AHRS_Lon = lon ; _Ex_AHRS_Ins._AHRS_Alt = alt;}
    void Set_AHRS_LLH(void);
    const Location &Get_MTi_LLH() const {return AHRS_LOC;}

    void set_ahrs_type(uint8_t type){_Ex_AHRS_Ins._AHRS_Type = type;}
    uint8_t get_ahrs_type(void)const{return _Ex_AHRS_Ins._AHRS_Type;}

    void set_counter(uint32_t counter){_Ex_AHRS_Ins._Counter = counter;}
    uint32_t get_counter(void)const{return _Ex_AHRS_Ins._Counter;}

    void set_hdop(uint16_t hdop){_GPS_Data._hdop = hdop;}
    uint16_t get_hdop(void)const{return _GPS_Data._hdop;}

    void set_fixtype(uint16_t fixtype){_GPS_Data._fixtype = fixtype;}
    uint16_t get_fixtype(void)const{return _GPS_Data._fixtype;}

    void set_num_statellite(uint16_t num_statellite){_GPS_Data._num_statellite = num_statellite;}
    uint16_t get_num_statellite (void)const{return _GPS_Data._num_statellite;}

    void set_engine_speed_left(uint16_t engine_left){_Engine_speed._Engine_Left = engine_left;}
    uint16_t get_engine_speed_left(void)const{return _Engine_speed._Engine_Left;}

    void set_engine_speed_right(uint16_t engine_right){_Engine_speed._Engine_Right = engine_right;}
    uint16_t get_engine_speed_right(void)const{return _Engine_speed._Engine_Right;}

    void set_servo_pwm11(uint16_t Swashplate_11){_Servo_pwm._Swashplate_11 = Swashplate_11;}
    uint16_t get_servo_pwm11(void)const{return _Servo_pwm._Swashplate_11;}

    void set_servo_pwm12(uint16_t Swashplate_12){_Servo_pwm._Swashplate_12 = Swashplate_12;}
    uint16_t get_servo_pwm12(void)const{return _Servo_pwm._Swashplate_12;}

    void set_servo_pwm13(uint16_t Swashplate_13){_Servo_pwm._Swashplate_13 = Swashplate_13;}
    uint16_t get_servo_pwm13(void)const{return _Servo_pwm._Swashplate_13;}

    void set_servo_pwm21(uint16_t Swashplate_21){_Servo_pwm._Swashplate_21 = Swashplate_21;}
    uint16_t get_servo_pwm21(void)const{return _Servo_pwm._Swashplate_21;}

    void set_servo_pwm22(uint16_t Swashplate_22){_Servo_pwm._Swashplate_22 = Swashplate_22;}
    uint16_t get_servo_pwm22(void)const{return _Servo_pwm._Swashplate_22;}

    void set_servo_pwm23(uint16_t Swashplate_23){_Servo_pwm._Swashplate_23 = Swashplate_23;}
    uint16_t get_servo_pwm23(void)const{return _Servo_pwm._Swashplate_23;}

    void set_tilt_angle(uint16_t tilt_angle){_Servo_pwm._Tilt_Angle = tilt_angle;}
    uint16_t get_tilt_angle(void)const{return _Servo_pwm._Tilt_Angle;}


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


    uint16_t checksum; //校验和
    uint8_t read_num;            //读到的数据个数
    uint8_t p_data;               //数据id的个数0-254;
    uint8_t buff[100];

   enum{
       PREAMBLE1 = 0, //帧头1
       PREAMBLE2,     //帧头2
       DATA,          //数据
       CHECKSUM,      //校验和
       PACK_END1,     //帧尾1
       PACK_END2,     //帧尾2
   }Data_State;


   struct
   {
       uint16_t hdop;//水平精度因子
       uint8_t fixtype;//定位状态
       uint8_t num_statellite;//卫星数量
   }GPS_Data; //单独加一个日志记录

   struct
   {
       uint16_t _hdop;//水平精度因子
       uint8_t _fixtype;//定位状态
       uint8_t _num_statellite;//卫星数量
   }_GPS_Data;


   /*用来存储外部惯导的数据(此时的数据还不确定是否正确)*/
   struct
   {
       Vector3f AHRS_Gyr;//rad/s
       Vector3f AHRS_Acc;//m/s^2
       Vector3f AHRS_Eulers;//rad
       Vector3f AHRS_Velocity;//m/s
       int32_t  AHRS_Lat;//*10e7
       int32_t  AHRS_Lon;//10e7
       float    AHRS_Alt;//m
       uint8_t  AHRS_Type;//传感器类型(1,2,3)
       uint32_t Counter;//接到的数据帧个数
   }Ex_AHRS_Ins;
   //用于存储已经确定正确的数据
   struct
   {
       Vector3f _AHRS_Gyr;//rad/s
       Vector3f _AHRS_Acc;//m/s^2
       Vector3f _AHRS_Eulers;//rad //需要强制转换成float
       Vector3f _AHRS_Velocity;//m/s
       int32_t  _AHRS_Lat;//*10e7
       int32_t  _AHRS_Lon;//10e7
       float    _AHRS_Alt;//m
       uint8_t  _AHRS_Type;//传感器类型(1,2,3)
       uint32_t _Counter;//接到的数据帧个数
   }_Ex_AHRS_Ins;

   //用来存储电机转速
   struct
   {
       uint16_t Engine_Left;//左电机转速
       uint16_t Engine_Right;//右电机转速
   }Engine_speed;

   //用来存储已确定正确的电机转速
   struct
   {
       uint16_t _Engine_Left;//左电机转速
       uint16_t _Engine_Right;//右电机转速
   }_Engine_speed;

   //舵机反馈数据
   struct
   {
       uint16_t Swashplate_11;//左自动倾斜器11(1000 - 2000)
       uint16_t Swashplate_12;//左自动倾斜器12
       uint16_t Swashplate_13;//左自动倾斜器13
       uint16_t Swashplate_21;//右自动倾斜器21
       uint16_t Swashplate_22;//右自动倾斜器22
       uint16_t Swashplate_23;//右自动倾斜器23
       float    Tilt_Angle;   //中央倾转舵机(-10 -90°)
   }Servo_pwm;

   //确定正确的舵机反馈数据
   struct
   {
       uint16_t _Swashplate_11;//左自动倾斜器11(1000 - 2000)
       uint16_t _Swashplate_12;//左自动倾斜器12
       uint16_t _Swashplate_13;//左自动倾斜器13
       uint16_t _Swashplate_21;//右自动倾斜器21
       uint16_t _Swashplate_22;//右自动倾斜器22
       uint16_t _Swashplate_23;//右自动倾斜器23
       float    _Tilt_Angle;   //中央倾转舵机(-10 -90°)
   }_Servo_pwm;

   /* union PACKED {
         uint16_t messageid; ;  //用他的元素来取数据
         uint8_t data[28];          //用他来存帧头的数据
    } header;
*/
    struct Location AHRS_LOC;
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter


};
