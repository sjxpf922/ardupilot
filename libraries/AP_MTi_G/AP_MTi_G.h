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
#include <AP_Common/AP_Common.h>
#include "AP_Common/Location.h"

class AP_MTi_G {
public:
  //  friend class AP_AHRS_View;
    AP_MTi_G();

    /* Do not allow copies */
    AP_MTi_G(const AP_MTi_G &other) = delete;
    AP_MTi_G &operator=(const AP_MTi_G&) = delete;

    // init - perform required initialisation
    bool init();
    bool Read_Mti_AHRS(void);           //ͨ�����ڶ�MTi����������
    void Mti_ReceiveData(uint8_t temp); //���ν���mti������
    void Mti_Parsing(uint8_t Id,uint8_t * data,uint8_t Len);//����MTI����
    void Mtidata_push(void);

    void set_mti_acc(Vector3f mti_acce){ MTI_EKF._MTI_acce = mti_acce ;}
    Vector3f get_mti_acc(void)const{return MTI_EKF._MTI_acce;}

    void set_mti_gyr(Vector3f mti_gyr){MTI_EKF._MTI_Gyr = mti_gyr;}
    Vector3f get_mti_gyr(void)const{return MTI_EKF._MTI_Gyr;}

    void set_mti_Matrix( Matrix3f mti_matrix){MTI_EKF._MTI_Matrix = mti_matrix;}
    Matrix3f get_mti_Matrix(void)const{return MTI_EKF._MTI_Matrix;}

    void set_mti_velocity(Vector3f mti_velocity){MTI_EKF._MTI_Velocity = mti_velocity;}
    Vector3f get_mti_velocity(void)const{return MTI_EKF._MTI_Velocity;}

    void set_mti_location(int32_t mti_lat , int32_t mti_lon, double mti_alt ){MTI_EKF._MTI_Lat = mti_lat;MTI_EKF._MTI_Lon = mti_lon ; MTI_EKF._MTI_Alt = mti_alt;}
    const Location &Get_MTi_LLH() const {   return Mti_Loc; }

    void set_mti_pressure(double mti_pressure){MTI_EKF._MTI_pressure = mti_pressure;}
    double get_mti_pressure(void)const{return MTI_EKF._MTI_pressure;}

    void set_mti_gps_type(uint8_t mti_gps_type){MTI_EKF._fixtype = mti_gps_type;}
    uint8_t get_mti_fixtype(void) const {return MTI_EKF._fixtype;}   //����mti��gps״̬

    void printf_serial5(void);
    void Matrix_to_eulers(Vector3f &eulers,Matrix3f &mat)const;
    void Eulers_to_Matrix(float &roll,float &pitch,float &yaw,Matrix3f &mat)const;
    bool Get_MTi_Position_NE(Vector2f &posNE)const;
    bool Get_MTi_Position_D(float &posD)const;
    void Set_MTi_LLH(void);
    void Get_MTi_Vel(Vector3f &vel) const{  vel = MTI_EKF._MTI_Velocity;}

   private:
    uint16_t checksum;              //У���
    uint8_t MID;                   //Message identifier
    uint8_t MessLen;              //��Ч���ݳ���
    int readnum ;                 //������Ч���ݣ���DATA��ʼ���ĸ������ֽڣ�
    uint8_t p_data;               //����id�ĸ���0-254;
    uint8_t buff[255];              //
    uint8_t Data[255];            //���ڴ������������ Ȼ�����崦��
    uint8_t DataId ;              //����ID
    uint8_t Data_Len;             //ÿ�����͵����ݳ���
    enum{
            Mti_Matrix = 0,      //��ת����
            Accel             ,  //���ٶ�
            Turn_Rate         ,  //ת��
            Velocity          ,  //�ٶ�
            Altitude          ,  //�߶�
            Lng_Lat           ,  //��γ��
            Air_Pressure      ,  //��ѹ
            Tempeature        ,  //�¶�
            Gps_Data          ,  //GPS״̬
    };
    struct PACKED HEAR{
        enum{
            PREAMBLE1 = 0,
            BUSID,
            MESSAGEID,
            DATALENGTH,//�����ܳ���
            DATAID,
            DATALEN,  //ÿһ�����ݵĳ���
            DATA,
            CHECKSUM,
        }mti_state;//״̬��־λ
    } MTi;

 //�����洢�ս������������ݣ���ʱ����ȷ���������Ƿ���ȷ��
    struct  {
        Vector3f  MTI_acce;  //m/s^2  NED body
        Vector3f  MTI_Gyr;   //rad/s
        Matrix3f  MTI_Matrix;
        Vector3f  MTI_Velocity;//m/s  NED
        Vector3f  MTI_magn;
        int32_t   MTI_Lat;//*10e7
        int32_t   MTI_Lon;//*10e7
        double    MTI_Alt;//m
        double    MTI_pressure;
        float     MTI_temp;
    }MTI_ins;

//�����洢gps״̬����
    struct {
        uint8_t fixtype;       //��λ״̬ 0- No Fix;1-Dead Reckoning only;2- 2D-Fix;3-3D-Fix;4- GNSS + dead reckoning combined
        uint8_t num_satellite;//��������
        uint16_t gdop; //���ξ�������*100
        uint16_t pdop; //λ�þ�������*100
        uint16_t tdop; //ʱ�Ӿ�������*100
        uint16_t vdop; //��ֱ��������*100
        uint16_t hdop; //ˮƽ��������*100
    }MTi_gps;

//�����洢�����ͨ��֮�������(����ȷ��Ϊ��ȷ������)
    struct  {
        Vector3f  _MTI_acce;  //m/s^2  NED body
        Vector3f  _MTI_Gyr;   //rad/s
        Matrix3f  _MTI_Matrix;
        Vector3f  _MTI_Velocity;//m/s  NED ef
        Vector3f  _MTI_magn;
        int32_t   _MTI_Lat;//*10e7
        int32_t   _MTI_Lon;//*10e7
        double    _MTI_Alt;//cm
        double    _MTI_pressure;
        float     _MTI_temp;
        uint8_t   _fixtype;
    }MTI_EKF;

    struct Location Mti_Loc;
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter


};
