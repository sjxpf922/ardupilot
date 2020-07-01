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
class MTi_G_core;

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
    void Mtidata_push(void); //将私有成员的数据转到公用

    void set_mti_acc(Vector3f mti_acce){ MTI_EKF._MTI_acce = mti_acce ;}
    Vector3f get_mti_acc(void)const{return MTI_EKF._MTI_acce;}

    void set_mti_gyr(Vector3f mti_gyr){MTI_EKF._MTI_Gyr = mti_gyr;}
    Vector3f get_mti_gyr(void)const{return MTI_EKF._MTI_Gyr;}

    void set_mti_attitude(Vector3f mti_attitude){MTI_EKF._MTI_attitude = mti_attitude;}
    Vector3f get_mti_attitude(void)const{return MTI_EKF._MTI_attitude;}

    void set_mti_velocity(Vector3f mti_velocity){MTI_EKF._MTI_Velocity = mti_velocity;}
    Vector3f get_mti_velocity(void)const{return MTI_EKF._MTI_Velocity;}

    void set_mti_location(int32_t mti_lat , int32_t mti_lon, double mti_alt ){MTI_EKF._MTI_Lat = mti_lat;MTI_EKF._MTI_Lon = mti_lon ; MTI_EKF._MTI_Alt = mti_alt;}

    void set_mti_pressure(double mti_pressure){MTI_EKF._MTI_pressure = mti_pressure;}
    void Get_MTi_Loc(struct Location & loc)const;
    void printf_serial5(void);
    void getEulerAngles( Vector3f &eulers) const;
    void getRotationBodyToNED(Matrix3f &mat) const;
    struct  {
               Vector3f  _MTI_acce;  //m/s^2  NED body
               Vector3f  _MTI_Gyr;   //rad/s
               Vector3f  _MTI_attitude;//rad
               Vector3f  _MTI_Velocity;//m/s  NED ef
               Vector3f  _MTI_magn;
               int32_t   _MTI_Lat;//*10e7
               int32_t   _MTI_Lon;//*10e7
               double    _MTI_Alt;//cm
               double    _MTI_pressure;
               float     _MTI_temp;
           }  MTI_EKF;


   private:
    uint8_t primary;   // current primary core
    MTi_G_core *core = nullptr;
    //uint8_t mti_state;             //
    uint16_t checksum;              //校验和
    uint8_t MID;                   //Message identifier
    uint8_t MessLen;              //有效数据长度
    int readnum ;                 //所有有效数据（从DATA开始）的个数（字节）
    uint8_t p_data;               //数据id的个数0-254;
    uint8_t buff[255];              //
    uint8_t Data[255];            //用于存放收来的数据 然后整体处理
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
    struct PACKED HEAR{
        enum{
                PREAMBLE1 = 0,
                BUSID,
                MESSAGEID,
                DATALENGTH,//数据总长度
                DATAID,
                DATALEN,  //每一种数据的长度
                DATA,
                CHECKSUM,
            }mti_state;//状态标志位
    } MTi;
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
