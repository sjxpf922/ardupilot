
/* 
  TestUart _ ����pixhawk ���������ͨѶ  _by TongXueShi
*/
#define AP_SERIALMANAGER_TestUart_BAUD        115200
#define AP_SERIALMANAGER_TestUart_BUFSIZE_RX      64
#define AP_SERIALMANAGER_TestUart_BUFSIZE_TX      64

#include <AP_TestUart/AP_TestUart.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;




AP_TestUart::AP_TestUart(void)
{
    _protocol = AP_SerialManager::SerialProtocol_None; //Ĭ�ϳ�ʼ��Ϊû��
    _port = NULL;                                      //Ĭ�ϳ�ʼ��Ϊ��
    airspeed = 0.0;
}

/*
 * init - perform required initialisation
 */
bool AP_TestUart::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TestUart, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_TestUart; // FrSky D protocol (D-receivers)
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //Initialize the uart
        _port->begin(AP_SERIALMANAGER_TestUart_BAUD,AP_SERIALMANAGER_TestUart_BUFSIZE_RX, AP_SERIALMANAGER_TestUart_BUFSIZE_TX);
        return true;
    }
    return false;

}


float AP_TestUart:: Asc_to_f(volatile unsigned char *str)
{
    signed char temp, flag1, flag2;
    float value, count;
    flag1 = 1;
    flag2 = 0;
    value = 0;
    count = 1;
    temp = *str;
    while (((*str >= '0') && (*str <= '9')) || (*str == '-') || (*str == '.')) //���ֻ����Ƿ���
    {
        temp = *str++;   //��������ȼ���������
        if (temp == '-')
        {
            if (flag1)
                flag1 = -1;
            else
                return (0x00); //��������'-' �����Ч
        }
        else if (temp == '.')
        {
            flag2 = 1;
        }
        else
        {
            value = value * 10 + (temp & 0x0f);
            if (flag2)
                count *= 0.1f; //f��ʾ�����������ӵĻ���������Ĭ��0.1Ϊ˫����double����
        }
    }
    value *= count * flag1; //����λ
    return (value);
}
/*

 */
float AP_TestUart::loop(void)
{
    if(_protocol != AP_SerialManager::SerialProtocol_TestUart || _port ==NULL)  //����ɿز�û��ָ���ĸ��������ӣ��򷵻�false
        return false;
    int16_t numc = _port->available();
    uint8_t data[AP_SERIALMANAGER_TestUart_BUFSIZE_RX] = {0};
    int16_t i = 0;
    for(i = 0; i < numc; i ++)
    {
        data[i] = _port -> read();

    }

    return Asc_to_f(data);
}

bool AP_TestUart::senddata(int buff)
{
    if(_protocol != AP_SerialManager::SerialProtocol_TestUart || _port ==NULL)
        return false;
    _port -> write(buff);
    return true;
}

float AP_TestUart :: ChangeSpeed(void)
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



/*bool AP_TestUart :: read(void)
{

    if( _port ==NULL)  //����ɿز�û��ָ���ĸ��������ӣ��򷵻�false
            return false;

     while (_port->available() > 0) {
            uint8_t temp = _port->read();
             parse(temp);
        }
     return true;

}


bool AP_TestUart::parse(uint8_t temp)
{
    switch (UART_msg.hear_state)
    {
        default:
        case HEAR::PREAMBLE1:
         if(temp == UART_PREAMBLE1)
             UART_msg.hear_state = HEAR::PREAMBLE2;
         break;
        case HEAR::PREAMBLE2:
            if(temp == UART_PREAMBLE2)
                UART_msg.hear_state = HEAR::DATA;
            else
                UART_msg.hear_state = HEAR::PREAMBLE1;
            break;
        case HEAR::DATA:
            chartofloat.buff[3++] = temp;

    }


}
*/

