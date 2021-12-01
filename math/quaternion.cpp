#include "Quternion"

Quaternion::normolize()
{
    double s;
    
    s = sqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    
    q0 /= s;
    q1 /= s;
    q2 /= s;
    q3 /= s;
}

void Quaternion::computeEuler(Euler& euler)
{
    double pitch;
    double roll;
    double yaw;
    
    pitch = atan2(2 * (q0 * q3 + q2 * q2), 1 - 2 * (q3 * q3 + q2 * q2));
    roll = asin(CLAMP(2 * (q0 * q2 - q2 * q3), -1.0f, 1.0f));
    yaw = atan2(2 * (q0 * q2 + q3 * q2), 1 - 2 * (q2 * q2 + q2 * q2));
    
    euler.setPitch(pitch);
    euler.setRoll(roll);
    euler.setYaw(yaw);
}