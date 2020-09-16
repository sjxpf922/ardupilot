
/* 
  外部惯性导航模块解码段
*/
#define AP_SERIALMANAGER_EX_AHRS_BAUD        460800
#define AP_SERIALMANAGER_EX_AHRS_BUFSIZE_RX      120
#define AP_SERIALMANAGER_EX_AHRS_BUFSIZE_TX      120
#define deg_to_rad 0.017453293f

#include <AP_EX_AHRS/AP_EX_AHRS.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;



//construct
AP_EX_AHRS::AP_EX_AHRS(void):
    checksum(0),
    read_num(0),
    p_data(0)
{
    _protocol = AP_SerialManager::SerialProtocol_None; //默认初始化为没有
    _port = NULL;                                      //默认初始化为空
    airspeed = 0.0;
    Data_State = PREAMBLE1;                   //将mti状态初始化为帧头1

}

 /* init - perform required initialisation*/


bool AP_EX_AHRS::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_EX_AHRS, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_EX_AHRS; // FrSky D protocol (D-receivers)
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //Initialize the uart
        _port->begin(AP_SERIALMANAGER_EX_AHRS_BAUD,AP_SERIALMANAGER_EX_AHRS_BUFSIZE_RX, AP_SERIALMANAGER_EX_AHRS_BUFSIZE_TX);
        return true;
    }
    return false;

}

bool AP_EX_AHRS :: Read_Ex_AHRS(void)
{
    if(_port == NULL)
        return false;
    uint32_t num = _port->available();
    while(num)
    {
        uint8_t data = _port -> read();
        Ex_ReceiveData(data);
        num --;
    }
    return true;
}

/* 依次接收外部惯导数据*/
void AP_EX_AHRS :: Ex_ReceiveData(uint8_t temp)
{
    if(read_num >= 66)
    {
        Data_State = PREAMBLE1;
        read_num = 0;
        Data_Push();
    }
    switch(Data_State)
    {
        case PREAMBLE1:
            if(temp == 0xFB)
            {
                Data_State = PREAMBLE2;
            }
            else
            {
                Data_State = PREAMBLE1;
            }
            break;

        case PREAMBLE2:
            if(temp == 0xFC)
            {
                Data_State = DATA;
                checksum = 0;
            }
            else
            {
                Data_State = PREAMBLE1;
            }
            break;

        case DATA:
            checksum += temp;
            read_num += 1;
            buff[p_data++] = temp;
            if(p_data == 63)//有效数据字节
            {
                Data_Parsing(buff);
                p_data = 0;
                Data_State = CHECKSUM;
            }
            break;

        case CHECKSUM:
            if (temp== (checksum & 0xff))
            {
                Data_State = PACK_END1;
                read_num += 1;
            }
            else
            {
                Data_State = PREAMBLE1;
                read_num = 0;
                checksum = 0;
            }
            break;

        case PACK_END1:
            if(temp == 0xA1)
            {
                Data_State = PACK_END2;
                read_num += 1;
            }
            else
            {
                Data_State = PREAMBLE1;
                read_num = 0;
                checksum = 0;
            }
            break;
        case PACK_END2:
            if(temp == 0xA2)
            {
                read_num += 1;
            }
            else
            {
                Data_State = PREAMBLE1;
                read_num = 0;
                checksum = 0;
            }
            break;
    }
}
//解码
void AP_EX_AHRS::Data_Parsing(uint8_t * data)
{
    union PACKED
    {
        uint8_t buff1[2];
        int16_t  data1;
    } Data1;               //用来解析int16_t数据

    union PACKED
    {
        uint8_t buff2[4];
        uint32_t  data2;
    } Data2;               //用来解析int32_t数据

    int n = 0,m = 0,k = 0;
    int32_t fchar[15];

    Ex_AHRS_Ins.AHRS_Type = data[0];//传感器类型
    for(uint8_t i = 0; i < 1; i ++)
    {
        for(uint8_t j = 0; j <= 3; j++)
        {
            Data2.buff2[3 - j] = *(data + 1 + n);
            n++;
        }
        fchar[m++] = Data2.data2;
    }
    Ex_AHRS_Ins.Counter = fchar[k++];//收到的数据帧个数

    n = 0;m = 0;k = 0;
    for(uint8_t i = 0; i < 12; i++)
    {
        for(uint8_t j = 0; j < 2; j++)
        {
           Data1.buff1[1 - j] = *(data + 5 + n);//先默认为大端
           n++;
        }
        fchar[m++] = Data1.data1;
    }
    Ex_AHRS_Ins.AHRS_Gyr.x = (float)(fchar[k++]*0.01f*deg_to_rad); //gyro
    Ex_AHRS_Ins.AHRS_Gyr.y = (float)(fchar[k++]*0.01f*deg_to_rad);
    Ex_AHRS_Ins.AHRS_Gyr.z = (float)(fchar[k++]*0.01f*deg_to_rad);

    Ex_AHRS_Ins.AHRS_Acc.x = (float)(fchar[k++]*0.01f); //acceleration
    Ex_AHRS_Ins.AHRS_Acc.y = (float)(fchar[k++]*0.01f);
    Ex_AHRS_Ins.AHRS_Acc.z = (float)(fchar[k++]*0.01f);

    Ex_AHRS_Ins.AHRS_Eulers.x = (float)(fchar[k++]*0.01f*deg_to_rad);//eulers
    Ex_AHRS_Ins.AHRS_Eulers.y = (float)(fchar[k++]*0.01f*deg_to_rad);
    Ex_AHRS_Ins.AHRS_Eulers.z = (float)(fchar[k++]*0.01f*deg_to_rad);

    Ex_AHRS_Ins.AHRS_Velocity.x = (float)(fchar[k++]*0.01f);//velocity
    Ex_AHRS_Ins.AHRS_Velocity.y = (float)(fchar[k++]*0.01f);
    Ex_AHRS_Ins.AHRS_Velocity.z = (float)(fchar[k++]*0.01f);

    n = 0;m = 0;k = 0;
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j <= 3; j++)
        {
           Data2.buff2[3 - j] = *(data + 29 + n);//先默认为大端
           n++;
        }
        fchar[m++] = Data2.data2;
    }
    Ex_AHRS_Ins.AHRS_Lat = fchar[k++];//纬度
    Ex_AHRS_Ins.AHRS_Lon = fchar[k++];//经度
    Ex_AHRS_Ins.AHRS_Alt = (float)(fchar[k++]*0.001f);//高度m

    n = 0;m = 0;k = 0;
    for(uint8_t i = 0; i < 10; i++)
    {
        for(uint8_t j = 0; j < 2; j++)
        {
           Data1.buff1[1 - j] = *(data + 41 + n);//先默认为大端
           n++;
        }
        fchar[m++] = Data1.data1;
    }
    Engine_speed.Engine_Left  = fchar[k++]; //左电机转速
    Engine_speed.Engine_Right = fchar[k++];//右电机转速

    Servo_pwm.Swashplate_11 = fchar[k++];//左自动倾斜器11(1000 - 2000)
    Servo_pwm.Swashplate_12 = fchar[k++];//左自动倾斜器12(1000 - 2000)
    Servo_pwm.Swashplate_13 = fchar[k++];//左自动倾斜器13(1000 - 2000)
    Servo_pwm.Swashplate_21 = fchar[k++];//右自动倾斜器21(1000 - 2000)
    Servo_pwm.Swashplate_22 = fchar[k++];//右自动倾斜器22(1000 - 2000)
    Servo_pwm.Swashplate_23 = fchar[k++];//右自动倾斜器23(1000 - 2000)
    Servo_pwm.Tilt_Angle = (float)(fchar[k++] * 0.01f );//中央倾转舵机(-10 -90°)

    GPS_Data.hdop = fchar[k++];//水平精度因子

    GPS_Data.num_statellite = data[61];//卫星数量
    GPS_Data.fixtype = data[62];       //gps定位状态
}
//只有校验通过了才会将值传出去
void AP_EX_AHRS::Data_Push(void)
{
    set_ahrs_gyr(Ex_AHRS_Ins.AHRS_Gyr);
    set_ahrs_acc(Ex_AHRS_Ins.AHRS_Acc);
    set_ahrs_eulers(Ex_AHRS_Ins.AHRS_Eulers);
    set_ahrs_velocity(Ex_AHRS_Ins.AHRS_Velocity);
    set_ahrs_location(Ex_AHRS_Ins.AHRS_Lat,Ex_AHRS_Ins.AHRS_Lon,Ex_AHRS_Ins.AHRS_Alt);
    Set_AHRS_LLH();
    set_ahrs_type(Ex_AHRS_Ins.AHRS_Type);
    set_counter(Ex_AHRS_Ins.Counter);
    set_hdop(GPS_Data.hdop);
    set_fixtype(GPS_Data.fixtype);
    set_num_statellite(GPS_Data.num_statellite);
    set_engine_speed_left(Engine_speed.Engine_Left);
    set_engine_speed_right(Engine_speed.Engine_Right);
    set_servo_pwm11(Servo_pwm.Swashplate_11);
    set_servo_pwm12(Servo_pwm.Swashplate_12);
    set_servo_pwm13(Servo_pwm.Swashplate_13);
    set_servo_pwm21(Servo_pwm.Swashplate_21);
    set_servo_pwm22(Servo_pwm.Swashplate_22);
    set_servo_pwm23(Servo_pwm.Swashplate_23);
    set_tilt_angle(Servo_pwm.Tilt_Angle);
}

/* set mti的经纬高 */
void AP_EX_AHRS :: Set_AHRS_LLH(void)
{
    AHRS_LOC.alt = (int32_t)(Ex_AHRS_Ins.AHRS_Alt*100.f);//转成厘米
    AHRS_LOC.lat = _Ex_AHRS_Ins._AHRS_Lat;
    AHRS_LOC.lng = _Ex_AHRS_Ins._AHRS_Lon;
}

/* 串口打印 调试时才调用 */
void AP_EX_AHRS :: printf_serial5(void)
{
    static int num = 0;
    num ++;
    if(num >= 250)
    {
        hal.uartF->printf("g:%f %f %f\n a:%f %f %f\n e:%f %f %f\n",_Ex_AHRS_Ins._AHRS_Gyr.x,_Ex_AHRS_Ins._AHRS_Gyr.y,_Ex_AHRS_Ins._AHRS_Gyr.z,_Ex_AHRS_Ins._AHRS_Acc.x,_Ex_AHRS_Ins._AHRS_Acc.y,_Ex_AHRS_Ins._AHRS_Acc.z,_Ex_AHRS_Ins._AHRS_Eulers.x,_Ex_AHRS_Ins._AHRS_Eulers.y,_Ex_AHRS_Ins._AHRS_Eulers.z);
        hal.uartF->printf("%d\n",_GPS_Data._fixtype);
        hal.uartF->printf("%d %d\n",_Engine_speed._Engine_Left,_Engine_speed._Engine_Right);
        hal.uartF->printf("%f\n",_Servo_pwm._Tilt_Angle);
        num = 0;
    }
}
/*
float AP_EX_AHRS :: ChangeSpeed(void)
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
*/
