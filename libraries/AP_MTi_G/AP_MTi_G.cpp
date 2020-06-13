
/* 
  TestUart _ 测试pixhawk 串口与电脑通讯  _by TongXueShi
*/
#define AP_SERIALMANAGER_TestUart_BAUD        115200
#define AP_SERIALMANAGER_TestUart_BUFSIZE_RX      64
#define AP_SERIALMANAGER_TestUart_BUFSIZE_TX      64

#include <AP_MTi_G/AP_MTi_G.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;



//construct
AP_MTi_G::AP_MTi_G(void) :
       mti_state(0),
       checksum(0),
       MID(0),
       MessLen(0),
       readnum (0),
       p_data(0),
       DataId(0),
       Data_Len(0),
       mti_register(0)
{
    _protocol = AP_SerialManager::SerialProtocol_None; //默认初始化为没有
    _port = NULL;                                      //默认初始化为空

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
        _port->begin(AP_SERIALMANAGER_TestUart_BAUD,AP_SERIALMANAGER_TestUart_BUFSIZE_RX, AP_SERIALMANAGER_TestUart_BUFSIZE_TX);
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
        Mti_ReceiveData(data);
        num --;
    }

    return true;
}

void AP_MTi_G :: Mti_ReceiveData(uint8_t temp)
{

    switch(mti_state)
    {
        case 0:
           if(temp == 0XFA)
           {
               mti_state = 1;
               checksum = 0;
           }
           break;
        case 1 :
            if(temp == 0xFF)
            {
                mti_state = 2;
                checksum +=temp;
            }
            else
            {
                if(temp == 0xFA)
                {
                    mti_state = 1;
                    checksum = 0;
                }
                else
                    {
                        mti_state = 0;
                        checksum = 0;
                    }
            }
            break;
        case 2 :
            MID = temp;
            if(MID!= 0x36)
            {
                mti_state = 0;
                checksum = 0;
            }
            mti_state = 3;
            checksum += temp;
            break;
        case 3:
            MessLen = temp;
            mti_state = 4;
            checksum += temp;
            p_data = 0;
            break;
        case 4:
            checksum += temp;
            readnum += 1;       //开始接收有效数据
            buff[p_data++] = temp;
            if(p_data == 2)
            {
                if(buff[0] == 0x20&& buff[1] == 0x30 )
                {
                    DataId = Attitude_Angle;
                    mti_state = 5;
                    p_data = 0;
                }
                else if(buff[0] == 0x40&& buff[1] == 0x20)
                {
                    DataId = Accel;
                    mti_state = 5;
                    p_data = 0;
                }
                else if(buff[0] == 0x80&& buff[1] == 0x20)
                {
                    DataId = Turn_Rate;
                    mti_state = 5;
                    p_data = 0;
                }
                else if(buff[0] == 0xD0&& buff[1] == 0x13)
                {
                   DataId = Velocity;
                   mti_state = 5;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x23)
                {
                   DataId = Altitude;
                   mti_state = 5;
                   p_data = 0;
                }
                else if(buff[0] == 0x50&& buff[1] == 0x43)
                {
                   DataId = Lng_Lat;
                   mti_state = 5;
                   p_data = 0;
                }
                else if(buff[0] == 0x30&& buff[1] == 0x10)
                {
                   DataId = Air_Pressure;
                   mti_state = 5;
                   p_data = 0;
                }
                else if(buff[0] == 0x08&& buff[1] == 0x10)
                {
                   DataId = Tempeature;
                   mti_state = 5;
                   p_data = 0;
                }
                else
                    p_data = 0;
            }
            break;
        case 5:
            readnum += 1;
            checksum += temp;
            mti_state = 6;
            Data_Len = temp;
            break;
        case 6:
            readnum += 1;
            checksum += temp;
            buff[p_data++] = temp;
            if(p_data >= Data_Len)
            {

                Mti_Parsing(DataId,buff,Data_Len);
                p_data = 0;
                mti_state = 4;
            }
            break;
        default:
                break;
    }
    if(readnum >= MessLen)  //数据接收并解析完成
    {
        mti_state = 0;
        readnum = 0;
        if(checksum&&0xff == 0)
        {
            //push_messages();//还没写
        }


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
            MTI_ins.MTI_attitude.x= fchar[m++]*DEG_TO_RAD_MTI;  //fchar[0]  这样就取出了8个  最多才放18个，现在要取出 8*3？
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
            mti_state = 0; //暂时不理解
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



int AP_MTi_G::wrap_360_cd_yaw(int yaw_change)
{
    if(yaw_change<=0) return -yaw_change;
    else return 360-yaw_change;
}
