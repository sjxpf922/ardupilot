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

//  Novatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_NOVA.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define NOVA_DEBUGGING 0

#if NOVA_DEBUGGING
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif
//构造函数
AP_GPS_NOVA::AP_GPS_NOVA(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    nova_msg.nova_state = nova_msg_parser::PREAMBLE1;

    const char *init_str = _initialisation_blob[0];
    const char *init_str1 = _initialisation_blob[1];
    
    port->write((const uint8_t*)init_str, strlen(init_str));
    port->write((const uint8_t*)init_str1, strlen(init_str1));
}

// Process all bytes available from the stream
//
bool
AP_GPS_NOVA::read(void)
{
    uint32_t now = AP_HAL::millis();

    if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
        const char *init_str = _initialisation_blob[_init_blob_index];

        if (now > _init_blob_time) {
            port->write((const uint8_t*)init_str, strlen(init_str));
            _init_blob_time = now + 200;
            _init_blob_index++;
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
    }
    
    return ret;
}

bool
AP_GPS_NOVA::parse(uint8_t temp)
{
    switch (nova_msg.nova_state)
    {
        default:
        case nova_msg_parser::PREAMBLE1:
            if (temp == NOVA_PREAMBLE1)
                nova_msg.nova_state = nova_msg_parser::PREAMBLE2;
            nova_msg.read = 0;
            break;
        case nova_msg_parser::PREAMBLE2:
            if (temp == NOVA_PREAMBLE2)
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE3;
            }
            else
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1; //返回帧头1重新开始
            }
            break;
        case nova_msg_parser::PREAMBLE3:
            if (temp == NOVA_PREAMBLE3)
            {
                nova_msg.nova_state = nova_msg_parser::HEADERLENGTH;
            }
            else
            {
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1; //否则重新从帧头1开始
            }
            break;
        case nova_msg_parser::HEADERLENGTH:
            Debug("NOVA HEADERLENGTH\n");
            nova_msg.header.data[0] = NOVA_PREAMBLE1;
            nova_msg.header.data[1] = NOVA_PREAMBLE2;
            nova_msg.header.data[2] = NOVA_PREAMBLE3;//前三个字节存储帧头 从结构体的排序可以看出
            nova_msg.header.data[3] = temp;          //第四个字节为帧头
            nova_msg.header.nova_headeru.headerlength = temp;//帧头长度
            nova_msg.nova_state = nova_msg_parser::HEADERDATA;
            nova_msg.read = 4;  //读了四个数据
            break;
        case nova_msg_parser::HEADERDATA:
            if (nova_msg.read >= sizeof(nova_msg.header.data)) {  //说明解析头溢出长度
                Debug("parse header overflow length=%u\n", (unsigned)nova_msg.read);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1; //重新找帧头
                break;
            }
            nova_msg.header.data[nova_msg.read] = temp;//从data[4]开始存储以后要解析的数据 第五个数据 数据id
            nova_msg.read++;
            if (nova_msg.read >= nova_msg.header.nova_headeru.headerlength) //说明已经进入有效数据接收了
            {
                nova_msg.nova_state = nova_msg_parser::DATA;
            }
            break;
        case nova_msg_parser::DATA:
            if (nova_msg.read >= sizeof(nova_msg.data)) {  //超过了能存储的空间
                Debug("parse data overflow length=%u msglength=%u\n", (unsigned)nova_msg.read,nova_msg.header.nova_headeru.messagelength);
                nova_msg.nova_state = nova_msg_parser::PREAMBLE1; //重新找帧头
                break;
            }
            nova_msg.data.bytes[nova_msg.read - nova_msg.header.nova_headeru.headerlength] = temp;//将数据存放在union中
            nova_msg.read++;      //在这里可不止接收一字节有效数据，下面有个break，会在这两行代码一直循环的，直到下面的if成立，开始进行校验
            if (nova_msg.read >= (nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength))//开始校验
            {
                Debug("NOVA DATA exit\n");
                nova_msg.nova_state = nova_msg_parser::CRC1;
            }
            break;
        case nova_msg_parser::CRC1:
            nova_msg.crc = (uint32_t) (temp << 0);//低八位
            nova_msg.nova_state = nova_msg_parser::CRC2;
            break;
        case nova_msg_parser::CRC2:
            nova_msg.crc += (uint32_t) (temp << 8);
            nova_msg.nova_state = nova_msg_parser::CRC3;
            break;
        case nova_msg_parser::CRC3:
            nova_msg.crc += (uint32_t) (temp << 16);
            nova_msg.nova_state = nova_msg_parser::CRC4;
            break;
        case nova_msg_parser::CRC4:
            nova_msg.crc += (uint32_t) (temp << 24);//高八位
            nova_msg.nova_state = nova_msg_parser::PREAMBLE1;//开始下一帧数据

            uint32_t crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.headerlength, (uint8_t *)&nova_msg.header.data, (uint32_t)0);
            crc = CalculateBlockCRC32((uint32_t)nova_msg.header.nova_headeru.messagelength, (uint8_t *)&nova_msg.data, crc);

            if (nova_msg.crc == crc)
            {
                return process_message(); //开始进行数据解析了
            }
            else
            {
                Debug("crc failed");
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool
AP_GPS_NOVA::process_message(void)
{
    uint16_t messageid = nova_msg.header.nova_headeru.messageid;//取出值

    Debug("NOVA process_message messid=%u\n",messageid);

    check_new_itow(nova_msg.header.nova_headeru.tow, nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength);
    
    if (messageid == 42) // bestpos  //说明这一帧是位置信息 进行相关数据的提取
    {
        const bestpos &bestposu = nova_msg.data.bestposu;  //引用  bestposu是nova_msg.data.bestposu的引用

        state.time_week = nova_msg.header.nova_headeru.week;//把周取出来
        state.time_week_ms = (uint32_t) nova_msg.header.nova_headeru.tow;//一周的时间
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = (int32_t) (bestposu.lat * (double)1e7);  //计算纬度
        state.location.lng = (int32_t) (bestposu.lng * (double)1e7);  //经度
        state.location.alt = (int32_t) (bestposu.hgt * 100);          //高度       这些值会被放在GPS_Backend.h public里供外部使用

        state.num_sats = bestposu.svsused;  //使用的卫星数量

        state.horizontal_accuracy = (float) ((bestposu.latsdev + bestposu.lngsdev)/2);  //水平均方根精度估计，单位为m
        state.vertical_accuracy = (float) bestposu.hgtsdev;  //垂直均方根精度估计，单位为m
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.rtk_age_ms = bestposu.diffage * 1000;   //最后一次基线校正的GPS年龄(毫秒)(没有校正时为0,0xFFFFFFFF表示溢出)
        state.rtk_num_sats = bestposu.svsused;

        if (bestposu.solstat == 0) // have a solution      //定位状态
        {
            switch (bestposu.postype)
            {
                case 16:
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 50: // narrow int
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                    state.status = AP_GPS::NO_FIX;
                    break;
            }
        }
        else
        {
            state.status = AP_GPS::NO_FIX;    //接收有效的GPS信息，但没有锁定
        }
        
        _new_position = true;           //标志位
    }

    if (messageid == 99) // bestvel      //说明是速度信息
    {
        const bestvel &bestvelu = nova_msg.data.bestvelu;

        state.ground_speed = (float) bestvelu.horspd;
        state.ground_course = (float) bestvelu.trkgnd;
        fill_3d_velocity();
        state.velocity.z = -(float) bestvelu.vertspd;
        state.have_vertical_velocity = true;
        
        _last_vel_time = (uint32_t) nova_msg.header.nova_headeru.tow;
        _new_speed = true;
    }

    if (messageid == 174) // psrdop    //说明这一帧数据是精度因子
    {
        const psrdop &psrdopu = nova_msg.data.psrdopu;

        state.hdop = (uint16_t) (psrdopu.hdop*100);
        state.vdop = (uint16_t) (psrdopu.htdop*100);
        return false;
    }

    // ensure out position and velocity stay insync
    if (_new_position && _new_speed && _last_vel_time == state.time_week_ms) {
        _new_speed = _new_position = false;
        
        return true;
    }
    
    return false;
}

void
AP_GPS_NOVA::inject_data(const uint8_t *data, uint16_t len)
{
    if (port->txspace() > len) {
        last_injected_data_ms = AP_HAL::millis();
        port->write(data, len);
    } else {
        Debug("NOVA: Not enough TXSPACE");
    }
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t AP_GPS_NOVA::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t AP_GPS_NOVA::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 )
    {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}
