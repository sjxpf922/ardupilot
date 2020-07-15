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
  //  friend class AP_AHRS_View;
    AP_MTi_G();

    /* Do not allow copies */
    AP_MTi_G(const AP_MTi_G &other) = delete;
    AP_MTi_G &operator=(const AP_MTi_G&) = delete;

    // init - perform required initialisation
    bool init();
    bool Read_Mti_AHRS(void);
    void Mti_ReceiveData(uint8_t temp);
    void Mti_Parsing(uint8_t Id,uint8_t * data,uint8_t Len);
    void Mtidata_push(void); //��˽�г�Ա������ת������

    void set_mti_acc(Vector3f mti_acce){ MTI_EKF._MTI_acce = mti_acce ;}
    Vector3f get_mti_acc(void)const{return MTI_EKF._MTI_acce;}

    void set_mti_gyr(Vector3f mti_gyr){MTI_EKF._MTI_Gyr = mti_gyr;}
    Vector3f get_mti_gyr(void)const{return MTI_EKF._MTI_Gyr;}

    //void set_mti_attitude(Vector3f mti_attitude){MTI_EKF._MTI_attitude = mti_attitude;}
   // Vector3f get_mti_attitude(void)const{return MTI_EKF._MTI_attitude;}

    void set_mti_Matrix( Matrix3f mti_matrix){MTI_EKF._MTI_Matrix = mti_matrix;}
    Matrix3f get_mti_Matrix(void)const{return MTI_EKF._MTI_Matrix;}

    void set_mti_velocity(Vector3f mti_velocity){MTI_EKF._MTI_Velocity = mti_velocity;}
    Vector3f get_mti_velocity(void)const{return MTI_EKF._MTI_Velocity;}

    void set_mti_location(int32_t mti_lat , int32_t mti_lon, double mti_alt ){MTI_EKF._MTI_Lat = mti_lat;MTI_EKF._MTI_Lon = mti_lon ; MTI_EKF._MTI_Alt = mti_alt;}

    void set_mti_pressure(double mti_pressure){MTI_EKF._MTI_pressure = mti_pressure;}
    void Get_MTi_Loc(struct Location & loc)const;
    void printf_serial5(void);
    //void getEulerAngles( Vector3f &eulers) const;
    void rotation_matrix(Matrix3f &m) const;
    void Matrix_to_eulers(Vector3f &eulers,Matrix3f &mat)const;
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
           }  MTI_EKF;
   private:
    uint8_t primary;   // current primary core
    //uint8_t mti_state;             //
    uint16_t checksum;              //У���
    uint8_t MID;                   //Message identifier
    uint8_t MessLen;              //��Ч���ݳ���
    int readnum ;                 //������Ч���ݣ���DATA��ʼ���ĸ������ֽڣ�
    uint8_t p_data;               //����id�ĸ���0-254;
    uint8_t buff[255];              //
    uint8_t Data[255];            //���ڴ������������ Ȼ�����崦��
    uint8_t DataId ;              //����ID
    uint8_t Data_Len;             //ÿ�����͵����ݳ���
    uint8_t  mti_register;        //���ڱ�ע��Ϣ����
    float  q1;
    float  q2;
    float  q3;
    float  q4;
    enum{
            //Attitude_Angle = 0,  //��̬��
            Mti_Matrix = 0,        //��Ԫ��
            Accel             ,  //���ٶ�
            Turn_Rate         ,  //ת��
            Velocity          ,  //�ٶ�
            Altitude          ,  //�߶�
            Lng_Lat           ,  //��γ��
            Air_Pressure      ,  //��ѹ
            Tempeature        ,  //�¶�
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
    struct  {
            Vector3f  MTI_acce;  //m/s^2  NED body
            Vector3f  MTI_Gyr;   //rad/s
            Matrix3f  MTI_Matrix;
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
