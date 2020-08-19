
/* 
  MTi驱动
*/
#define AP_SERIALMANAGER_MTi_G_BAUD            460800    //波特率
#define AP_SERIALMANAGER_MTi_G_BUFSIZE_RX      256       //接收缓冲区
#define AP_SERIALMANAGER_MTi_G_BUFSIZE_TX      256       //发送缓冲区

#include <AP_MTi_G/AP_MTi_G.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
extern const AP_HAL::HAL& hal;

//construct
AP_MTi_G::AP_MTi_G(void):
      checksum(0),
      readnum(0),
      p_data(0)
{
    _protocol = AP_SerialManager::SerialProtocol_None; //默认初始化协议为空
    _port = NULL;                                      //默认初始化串口指针为空
    MTi.mti_state = HEAR::PREAMBLE1;                   //将mti状态初始化为帧头1
}

 /* init - perform required initialisation*/

bool AP_MTi_G::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MTIG_700, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_MTIG_700; // FrSky D protocol (D-receivers)
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //Initialize the uart
        _port->begin(AP_SERIALMANAGER_MTi_G_BAUD,AP_SERIALMANAGER_MTi_G_BUFSIZE_RX, AP_SERIALMANAGER_MTi_G_BUFSIZE_TX);
        return true;
    }
    return false;
}

/* 读取MTi数据  */
bool AP_MTi_G :: Read_Mti_AHRS(void)
{
    if(_port == NULL)
        return false;
    uint32_t num = _port->available();
    while(num)
    {
        uint8_t data = _port -> read();
        Mti_ReceiveData(data);
        num --;
    }
    return true;
}

/* 依次接收MTi数据 */
void AP_MTi_G :: Mti_ReceiveData(uint8_t temp)
{
    if(readnum > MessLen)  //所有数据都接收、解析完毕，统一publish出去  注：应该是大于号。
    {
        MTi.mti_state = HEAR::PREAMBLE1;
        readnum = 0;
        if((checksum&0xff) == 0)
        {
            Mtidata_push();
          //  printf_serial5();
        }
    }
   switch(MTi.mti_state)
    {
       case HEAR::PREAMBLE1:
           if(temp == 0xFA)
           {
               MTi.mti_state = HEAR::BUSID;
               checksum = 0;
           }
           break;
        case HEAR::BUSID:
            if(temp == 0xFF)
            {
                MTi.mti_state = HEAR::MESSAGEID;
                checksum += temp;
            }
            else
            {
                if(temp == 0xFA)
                {
                    MTi.mti_state = HEAR::BUSID;
                   checksum = 0;
                }
                else
                {
                    MTi.mti_state = HEAR::PREAMBLE1;
                    checksum = 0;
                }
            }
            break;
        case HEAR::MESSAGEID :
            MID = temp;
            if(MID != 0x36)
            {
                MTi.mti_state = HEAR::PREAMBLE1;
                checksum = 0;
            }
            else
            {
                MTi.mti_state = HEAR::DATALENGTH;
                checksum += temp;
            }
            break;
        case HEAR::DATALENGTH:
            MessLen = temp;
            MTi.mti_state = HEAR::DATAID;
            checksum += temp;
            p_data = 0;
            break;
        case HEAR::DATAID:
            checksum += temp;
            readnum += 1;       //开始接收有效数据
            buff[p_data++] = temp;
            if(p_data == 2)
            {
                if(buff[0] == 0x20&& buff[1] == 0x20 )    //dcm矩阵
                {  
                    DataId = Mti_Matrix;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0x40&& buff[1] == 0x20) //加速度
                {  
                    DataId = Accel;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0x80&& buff[1] == 0x20) //角速度
                { 
                    DataId = Turn_Rate;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0xD0&& buff[1] == 0x13)  //速度
                { 
                   DataId = Velocity;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x23)  //高度
                { 
                   DataId = Altitude;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x43)  //经纬度
                { 
                   DataId = Lng_Lat;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x30&& buff[1] == 0x10)  //气压
                { 
                   DataId = Air_Pressure;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x08&& buff[1] == 0x10)  //温度
                { 
                   DataId = Tempeature;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x70&& buff[1] == 0x10)  //gps状态
                {
                   DataId = Gps_Data;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else
                    p_data = 0;
            }
            break;
        case HEAR::DATALEN:
            readnum += 1;
            checksum += temp;
            MTi.mti_state = HEAR::DATA;
            Data_Len = temp;
            break;
        case HEAR::DATA:
            readnum += 1;
            checksum += temp;
            buff[p_data++] = temp;
            if(p_data >= Data_Len)
             {
                Mti_Parsing(DataId,buff,Data_Len);
                p_data = 0;
                MTi.mti_state = HEAR::DATAID;
             }
               break;
        default:
                break;
    }

}

/* 对接收的数据解析 */
void AP_MTi_G :: Mti_Parsing(uint8_t ID,uint8_t * data,uint8_t Len)
{
    union PACKED mess_float
    {
        float Mti_a;
        uint8_t Mti_b[4];
        uint32_t  Mti_c;
    } MtiData;               //用来解析float数据

    union PACKED mess_double
    {
        double Mti_Data;
        uint8_t Mti_buff[8];
    } MtiData1;              //用来解析double数据

    union PACKED mess_U16
    {
        uint16_t Mti_Gps;
        uint8_t Mti_buff[2];
    } MtiData2;               //用来解析gps状态

    int n = 0,m = 0,data_length,mti_packet=0;
    double  fchar[18];  

    switch(ID)
    {
        case Mti_Matrix:
            data_length = Len/4;
            for(int i = 0;i < data_length;i ++)
            {
                for(int j = 0;j <=3 ;j ++)
                {
                    MtiData.Mti_b[3-j] = *(data+n); //平台为小端，传输为大端
                    n++;
                }
                fchar[mti_packet++] = MtiData.Mti_a;
            }
            MTI_ins.MTI_Matrix.a.x = fchar[m++];
            MTI_ins.MTI_Matrix.b.x = fchar[m++];
            MTI_ins.MTI_Matrix.c.x = fchar[m++];
            MTI_ins.MTI_Matrix.a.y = fchar[m++];
            MTI_ins.MTI_Matrix.b.y = fchar[m++];
            MTI_ins.MTI_Matrix.c.y = fchar[m++];
            MTI_ins.MTI_Matrix.a.z = fchar[m++];
            MTI_ins.MTI_Matrix.b.z = fchar[m++];
            MTI_ins.MTI_Matrix.c.z = fchar[m++];
            break;
        case Accel:

            data_length=Len/4;  
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j < 4; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++] = MtiData.Mti_a;
            }
            MTI_ins.MTI_acce.x = fchar[m++];
            MTI_ins.MTI_acce.y = fchar[m++];
            MTI_ins.MTI_acce.z = fchar[m++];
            break;
        case Turn_Rate:
            data_length=Len/4;   
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j < 4; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++] = MtiData.Mti_a;
            }
            MTI_ins.MTI_Gyr.x = fchar[m++];
            MTI_ins.MTI_Gyr.y = fchar[m++];
            MTI_ins.MTI_Gyr.z = fchar[m++];
            break;
        case Velocity:
            data_length=3;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 7; j ++)
                {
                    MtiData1.Mti_buff[7-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++] = MtiData1.Mti_Data;
            }
            MTI_ins.MTI_Velocity.x = fchar[m++];
            MTI_ins.MTI_Velocity.y = fchar[m++];
            MTI_ins.MTI_Velocity.z = fchar[m++];
            break;
        case Altitude:
            data_length=1;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 7; j ++)
                {
                    MtiData1.Mti_buff[7-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++] = MtiData1.Mti_Data;
            }
            MTI_ins.MTI_Alt = fchar[m++];
            break;
        case Lng_Lat:
            data_length=Len/8;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 7; j ++)
                {
                    MtiData1.Mti_buff[7-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++]=MtiData1.Mti_Data;
            }
            MTI_ins.MTI_Lat= (int32_t)(fchar[m++]*10000000.0f);
            MTI_ins.MTI_Lon= (int32_t)(fchar[m++]*10000000.0f);
            break;

        case Air_Pressure:
             data_length=Len/4;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 3; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet ++]=MtiData.Mti_c;
            }
            MTI_ins.MTI_pressure= fchar[m++];
            break;
        case Tempeature:
            data_length = Len/4;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 3; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++] = MtiData.Mti_a;
            }
            MTI_ins.MTI_temp = fchar[m++];
            break;
        case Gps_Data:
            MTi_gps.fixtype = *(data + 20);
            MTi_gps. num_satellite = *(data + 22);
            for(int i = 0;i < 5; i++)
            {
                for(int j = 0;j < 2;j++)
                {
                    MtiData2.Mti_buff[1-j] = *(data + 80 + n );
                    n++;
                }
                fchar[mti_packet++] = MtiData2.Mti_Gps;
            }
            MTi_gps.gdop = fchar[m++];
            MTi_gps.pdop = fchar[m++];
            MTi_gps.tdop = fchar[m++];
            MTi_gps.vdop = fchar[m++];
            MTi_gps.hdop = fchar[m++];
            break;
        default:
                break;
    }

}


/* 只有校验和通过了，才会将值传出去  */
void AP_MTi_G:: Mtidata_push(void)
{
    set_mti_acc(MTI_ins.MTI_acce);
    set_mti_gyr(MTI_ins.MTI_Gyr);
    set_mti_Matrix(MTI_ins.MTI_Matrix);
    set_mti_velocity(MTI_ins.MTI_Velocity);
    Set_MTi_LLH();
    set_mti_pressure(MTI_ins.MTI_pressure);
    set_mti_gps_type(MTi_gps.fixtype);
}

/* set mti的经纬高 */
void AP_MTi_G :: Set_MTi_LLH(void)
{
    Mti_Loc.alt = (int32_t)(MTI_ins.MTI_Alt*100.f);
    Mti_Loc.lat = MTI_ins.MTI_Lat;
    Mti_Loc.lng = MTI_ins.MTI_Lon;
}

/* 计算mti的xy轴的相对位置 */
bool AP_MTi_G::Get_MTi_Position_NE(Vector2f &posNE)const
{
    const struct Location &mtiloc = Get_MTi_LLH();
    Vector2f _posNE;
    if(mtiloc.get_vector_xy_from_origin_NE(_posNE))
    {
        posNE.x = _posNE.x * 0.01f;  //米
        posNE.y = _posNE.y * 0.01f;
        return true;
    }
    else
        return false;
}

/* 计算mti的z轴相对位置 */
bool AP_MTi_G::Get_MTi_Position_D(float &posD)const
{
    const struct Location &mtiloc = Get_MTi_LLH();
    float _posD;
    if(mtiloc.get_mti_xy_from_origin_D(_posD))
    {
        posD = _posD * 0.01f;  //米
        return true;
    }
    else
        return false;
}

/* 串口打印 调试时才调用 */
void AP_MTi_G :: printf_serial5(void)
{
    static int num = 0;
    num ++;
    if(num >= 250)
    {
        hal.uartF->printf("%f %f %f\n%f %f %f\n%f %f %f\n",MTI_ins.MTI_Matrix.a.x,MTI_ins.MTI_Matrix.a.y,MTI_ins.MTI_Matrix.a.z,MTI_ins.MTI_Matrix.b.x,MTI_ins.MTI_Matrix.b.y,MTI_ins.MTI_Matrix.b.z,MTI_ins.MTI_Matrix.c.x,MTI_ins.MTI_Matrix.c.y,MTI_ins.MTI_Matrix.c.z);
        num = 0;
    }
}

//dcm转eulers
void AP_MTi_G::Matrix_to_eulers(Vector3f &eulers,Matrix3f &mat)const
{
    mat.to_euler(&eulers.x,&eulers.y,&eulers.z);
}
