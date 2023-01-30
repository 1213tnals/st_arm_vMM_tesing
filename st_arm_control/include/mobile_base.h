#ifndef MOBILE_BASE_H
#define MOBILE_BASE_H

#include <eigen3/Eigen/Dense>

using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::Vector3d;

class Mobile_Base
{
public:
    Mobile_Base();
    ~Mobile_Base();

    Quaterniond joystick_quaternion;
    Vector3d joystick_position;
    VectorXd wheel_speed = VectorXd::Zero(4);         //wheel 4
    VectorXd ref_wheel_speed = VectorXd::Zero(4);     //wheel 4

    void GetWheelSpeed(VectorXd a_wheel_speed);
    void BaseMovingGeneration();
    

private:

};
#endif  // MOBILE_BASE_H