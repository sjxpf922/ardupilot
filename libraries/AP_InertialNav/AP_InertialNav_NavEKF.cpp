#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update()    //更新导航的速度与位置
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {       //调用AHRS_NavEKF
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU 将NED中的m转换成NEU里的cm
    }

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU北东地转到北东天
    }
}
/*
 void AP_InertialNav_NavEKF::update()；该函数通过调用  AP_AHRS_NavEKF.cpp里的方法，统一更新位置、速度信息
 下面的函数目的就是将获取的速度、位置分开，方便调用；
 const Vector3f &AP_InertialNav_NavEKF::get_position(void) const //位置
 const Vector3f &AP_InertialNav_NavEKF::get_velocity() const    //速度
 .......


*/

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const // 滤波器状态
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const //位置
{
    return _relpos_cm;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

/**
 * get_speed_xy - returns the current horizontal speed in cm/s
 *
 * @returns the current horizontal speed in cm/s
 */
float AP_InertialNav_NavEKF::get_speed_xy() const //水平速度
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const //高度
{
    return _relpos_cm.z;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const //爬升率
{
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
