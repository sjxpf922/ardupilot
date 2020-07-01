#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_AHRS;

class MTi_G_core
{
public :
	MTi_G_core (const AP_AHRS *ahrs);

	void getRotationBodyToNED(Matrix3f &mat) const;
private :
	 struct output_elements {
	        Quaternion  quat;           // 0..3
	        Vector3f    velocity;       // 4..6
	        Vector3f    position;       // 7..9
	    }outputDataNew;
const AP_AHRS *_ahrs;
};
