#ifndef _EULER_H
#define _EULER_H

class Quaternion;

class Euler {
public:
    Euler();
    Euler(const Quaternion &quaternion);
    ~Euler();

    void setRoll(double roll);
    double getRoll();
    void setPitch(double pitch);
    double getPitch();
    void setYaw(double yaw);
    double getYaw();

private:
    double m_roll;
    double m_pitch;
    double m_yaw;
};
 
#endif // _EULER_H