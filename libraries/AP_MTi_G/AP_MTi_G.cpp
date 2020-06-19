
/* 
  MTi驱动测试 pixhawk   _by TongXueShi
*/
#define AP_SERIALMANAGER_MTi_G_BAUD        115200
#define AP_SERIALMANAGER_MTi_G_BUFSIZE_RX      125
#define AP_SERIALMANAGER_MTi_G_BUFSIZE_TX      125

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
    _protocol = AP_SerialManager::SerialProtocol_None; //默认初始化为没有
    _port = NULL;                                      //默认初始化为空
    MTi.mti_state = HEAR::PREAMBLE1;
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

bool AP_MTi_G :: Read_Mti_AHRS(void)
{
    if(_port == NULL)
        return false;
    uint32_t num = _port->available();
    while(num)
    {
        uint8_t data = _port -> read();

      // hal.uartF->write(data);
       // hal.uartF->printf("num = %ld\n",num);
      Mti_ReceiveData(data);
       num --;

    }

    return true;
}

void AP_MTi_G :: Mti_ReceiveData(uint8_t temp)
{
   //hal.uartF->printf("temp = %x\n",temp);
   switch(MTi.mti_state)
    {

       case HEAR::PREAMBLE1:
           if(temp == 0xFA)
           {
               MTi.mti_state = HEAR::BUSID;
            //   hal.uartF->printf("1\n");
               checksum = 0;
             //  hal.uartF->printf("%d\n",MTi.mti_state);
           }
           break;
        case HEAR::BUSID:
          //  hal.uartF->printf("进入case2\n");
            if(temp == 0xFF)
            {
               // hal.uartF->printf("找到帧头2\n");
                MTi.mti_state = HEAR::MESSAGEID;
                checksum +=temp;
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
            if(MID!= 0x36)
            {
                MTi.mti_state = HEAR::PREAMBLE1;
                checksum = 0;
            }
            MTi.mti_state = HEAR::DATALENGTH;
            checksum += temp;
           // hal.uartF->printf("找到数据ID\n");
            break;
        case HEAR::DATALENGTH:
            //hal.uartF->printf("找到数据长度\n");
            MessLen = temp;
          //  hal.uartF->printf("MessLen = %d\n",MessLen);
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
                if(buff[0] == 0x20&& buff[1] == 0x30 )
                {  // hal.uartF->printf("找到姿态数据\n");
                    DataId = Attitude_Angle;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0x40&& buff[1] == 0x20)
                {  // hal.uartF->printf("找到加速度数据\n");
                    DataId = Accel;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0x80&& buff[1] == 0x20)
                { //hal.uartF->printf("找到角速度数据\n");
                    DataId = Turn_Rate;
                    MTi.mti_state = HEAR::DATALEN;
                    p_data = 0;
                }
                else if(buff[0] == 0xD0&& buff[1] == 0x13)
                { // hal.uartF->printf("找到速度数据\n");
                   DataId = Velocity;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x23)
                { // hal.uartF->printf("找到高度数据\n");
                   DataId = Altitude;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x43)
                { // hal.uartF->printf("找到经纬高数据\n");
                   DataId = Lng_Lat;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x30&& buff[1] == 0x10)
                { // hal.uartF->printf("找到气压数据\n");
                   DataId = Air_Pressure;
                   MTi.mti_state = HEAR::DATALEN;
                   p_data = 0;
                }
                else if(buff[0] == 0x08&& buff[1] == 0x10)
                { // hal.uartF->printf("找到温度数据\n");
                   DataId = Tempeature;
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
            if(p_data == Data_Len)
            {
               // hal.uartF->printf("进入checksum\n");
                Mti_Parsing(DataId,buff,Data_Len);
                p_data = 0;
                if(readnum == MessLen)
                {
                   // hal.uartF->printf("有效数据读完\n");
                    MTi.mti_state =  HEAR::CHECKSUM;
                    readnum = 0;
                }
                else
                {
                   // hal.uartF->printf("有效数据未读完继续读\n");
                    MTi.mti_state = HEAR::DATAID;
                }

            }
            break;
        case HEAR::CHECKSUM:
            checksum +=temp;
          //  hal.uartF->printf("%X\n",checksum);
           if((checksum&0xff) == 0)
            {
               // hal.uartF->printf("ok\n");
                MTi.mti_state = HEAR::PREAMBLE1;
                checksum = 0;
               // hal.uartF->printf("最后的数据处理 并进入下一帧\n");
                printf_data();
                printf_serial5();
            }
           else
           {
               hal.uartF->printf("flase\n");
               MTi.mti_state = HEAR::PREAMBLE1;
               checksum = 0;
               readnum = 0;
           }
           break;
        default:
            break;

    }

}

void AP_MTi_G :: Mti_Parsing(uint8_t ID,uint8_t * data,uint8_t Len)
{
    union PACKED mess_float
    {
        float Mti_a;
        uint8_t Mti_b[4];
        uint32_t  Mti_c;
    } MtiData;        //用来解析float数据

    union PACKED mess_double
        {
            double Mti_Data;
            uint8_t Mti_buff[8];
        } MtiData1;  //用来解析double数据
    int n=0,m=0,data_length,mti_packet=0;
    double  fchar[18];  //选择double类型，应该是考虑了部分数据的精度问题。
    switch(ID)
    {
        case Attitude_Angle:
            data_length=Len/4;   //x y z   data_length 是多少字节呢？
            for(int i=0;i<data_length;i++)
            {
                for(int j=0;j<=3;j++)
                {
                    MtiData.Mti_b[3-j]=*(data+n);//平台为小端，传输为大端
                    n++;
                }
                fchar[mti_packet++]=MtiData.Mti_a;    //循环把xyz的数据(已经是float数据了)存放在fchar[]
            }
            MTI_ins.MTI_attitude.x= fchar[m++]*DEG_TO_RAD_MTI;  //fchar[0]
            MTI_ins.MTI_attitude.y=-fchar[m++]*DEG_TO_RAD_MTI;  //fchar[1]
            MTI_ins.MTI_attitude.z=wrap_360_cd_yaw(fchar[m++]-90)*DEG_TO_RAD_MTI; //fchar[2]
            mti_register|=0x01;
           break;
        case Accel:

            data_length=Len/4;   //x y z
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j < 4; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++]=MtiData.Mti_a;
            }
            MTI_ins.MTI_acce.x= fchar[m++];
            MTI_ins.MTI_acce.y=-fchar[m++];
            MTI_ins.MTI_acce.z=-fchar[m++];
            mti_register|=0x02;
            break;
        case Turn_Rate:
            data_length=Len/4;   //x y z
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j < 4; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++]=MtiData.Mti_a;
            }
            MTI_ins.MTI_Gyr.x= fchar[m++];
            MTI_ins.MTI_Gyr.y=-fchar[m++];
            MTI_ins.MTI_Gyr.z=-fchar[m++];
            mti_register|=0x04;
           // MTi.mti_state = 0; //暂时不理解
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
                fchar[mti_packet++]=MtiData1.Mti_Data;
            }
            MTI_ins.MTI_Velocity.y= fchar[m++]; //注意此处方向与上面不同需要看
            MTI_ins.MTI_Velocity.x=-fchar[m++];
            MTI_ins.MTI_Velocity.z=-fchar[m++];
            mti_register|=0x08;
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
                fchar[mti_packet++]=MtiData1.Mti_Data;
            }
            MTI_ins.MTI_Alt= fchar[m++];
            mti_register|=0x10;
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
            mti_register|=0x20;

           /* gps_tick++;  //暂时不解
                        if(gps_tick==10)
                        {
                            gps.set_fix_time();
                            gps_tick=0;
                        }*/
            break;

        case Air_Pressure:
            data_length=1;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 3; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++]=MtiData.Mti_c;
            }
            MTI_ins.MTI_pressure= fchar[m++];

            mti_register|=0x40;
            break;
        case Tempeature:
            data_length = 1;
            for(int i = 0; i < data_length; i++)
            {
                for(int j = 0;j <= 3; j ++)
                {
                    MtiData.Mti_b[3-j] = *(data + n);
                    n++;
                }
                fchar[mti_packet++]=MtiData.Mti_a;
            }
            MTI_ins.MTI_pressure= fchar[m++];

            mti_register|=0x80;
            break;
        default:
                break;

    }

}

//向地面站发送数据
void AP_MTi_G::printf_data(void)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL," acc_x = %f\n acc_y = %f\n acc_z = %f\n",MTI_ins.MTI_acce.x,MTI_ins.MTI_acce.y,MTI_ins.MTI_acce.z);
    gcs().send_text(MAV_SEVERITY_CRITICAL," gyr_x = %f\n gyr_y = %f\n gyr_z = %f\n ",MTI_ins.MTI_Gyr.x,MTI_ins.MTI_Gyr.y,MTI_ins.MTI_Gyr.z);
    gcs().send_text(MAV_SEVERITY_CRITICAL," Alt = %lf\n speed_x = %f\n speed_y = %f\n speed_z = %f\n lat = %ld\n lon = %ld\n Press = %lf\n",MTI_ins.MTI_Alt,MTI_ins.MTI_Velocity.x,MTI_ins.MTI_Velocity.y,MTI_ins.MTI_Velocity.z,MTI_ins.MTI_Lat,MTI_ins.MTI_Lon,MTI_ins.MTI_pressure);

    gcs().send_text(MAV_SEVERITY_CRITICAL," roll = %f\n pitch = %f\n yew = %f\n",MTI_ins.MTI_attitude.x,MTI_ins.MTI_attitude.y,MTI_ins.MTI_attitude.z);
    gcs().send_text(MAV_SEVERITY_CRITICAL," T = %f\n",MTI_ins.MTI_temp);

}

//向串口发送数据
void AP_MTi_G :: printf_serial5(void)
{
    static int n =0;
    n++;
    if(n >=100)
    {
        hal.uartF->printf(" acc_x = %f\n acc_y = %f\n acc_z = %f\n",MTI_ins.MTI_acce.x,MTI_ins.MTI_acce.y,MTI_ins.MTI_acce.z);
        hal.uartF->printf(" gyr_x = %f\n gyr_y = %f\n gyr_z = %f\n ",MTI_ins.MTI_Gyr.x,MTI_ins.MTI_Gyr.y,MTI_ins.MTI_Gyr.z);
        hal.uartF->printf(" Alt = %lf\n speed_x = %f\n speed_y = %f\n speed_z = %f\n lat = %ld\n lon = %ld\n Press = %lf\n",MTI_ins.MTI_Alt,MTI_ins.MTI_Velocity.x,MTI_ins.MTI_Velocity.y,MTI_ins.MTI_Velocity.z,MTI_ins.MTI_Lat,MTI_ins.MTI_Lon,MTI_ins.MTI_pressure);
        hal.uartF->printf(" roll = %f\n pitch = %f\n yew = %f\n",MTI_ins.MTI_attitude.x,MTI_ins.MTI_attitude.y,MTI_ins.MTI_attitude.z);
        hal.uartF->printf(" T = %f\n",MTI_ins.MTI_temp);
        n=0;
    }
}
int AP_MTi_G::wrap_360_cd_yaw(int yaw_change)
{
    if(yaw_change<=0) return -yaw_change;
    else return 360-yaw_change;
}
