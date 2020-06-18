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
    void printf_data(void);
    int wrap_360_cd_yaw(int yaw_change);
    void  printf_serial5(void);
   private:

    static const uint8_t UART_PREAMBLE1 = 0xaa;  //����֡ͷ 1
    static const uint8_t UART_PREAMBLE2 = 0x44;    //2
    static const uint8_t DataId1 = 0x50;  //�ٶ�����ID
    static const uint8_t DataId2 = 0x55;  //�߶�����ID

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

    enum{
            Attitude_Angle = 0,  //��̬��
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
