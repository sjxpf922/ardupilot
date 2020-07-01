#include <AP_MTi_G/AP_MTi_G_core.h>

extern const AP_HAL::HAL& hal;

//construct
MTi_G_core::MTi_G_core(const AP_AHRS *ahrs) :
        _ahrs(ahrs)
{}

// return the transformation matrix from XYZ (body) to NED axes
void MTi_G_core::getRotationBodyToNED(Matrix3f &mat) const
{
    outputDataNew.quat.rotation_matrix(mat);
    mat = mat * _ahrs->get_rotation_vehicle_body_to_autopilot_body();
}
