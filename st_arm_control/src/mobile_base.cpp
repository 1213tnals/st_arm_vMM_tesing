#include "mobile_base.h"


Mobile_Base::Mobile_Base() {}
Mobile_Base::~Mobile_Base() {}


void Mobile_Base::GetWheelSpeed(VectorXd a_wheel_speed)
{
    for(uint8_t i=0; i<4; i++) wheel_speed[i] = a_wheel_speed[i];
}

void Mobile_Base::BaseMovingGeneration()
    {
        // ref_wheel_speed[0] = joystick_position.x();   //FL
        // ref_wheel_speed[1] = joystick_position.y();   //BL
        // ref_wheel_speed[2] = joystick_position.z();   //FR
        // ref_wheel_speed[3] = joystick_position.x();   //BR
        // I should make formula to make ref_wheel_speed by using joystick_posion.x,y,z

        // ref_speed[0] = speed[0];
        // just go straight
        ref_wheel_speed[0] = 1;   //FL
        ref_wheel_speed[1] = 1;   //BL
        ref_wheel_speed[2] = 1;   //FR
        ref_wheel_speed[3] = 1;   //BR

        // // just go back
        // ref_wheel_speed[0] = -1;
        // ref_wheel_speed[1] = -1;
        // ref_wheel_speed[2] = -1;
        // ref_wheel_speed[3] = -1;

        // // just go left
        // ref_wheel_speed[0] = -1;
        // ref_wheel_speed[1] = 1;
        // ref_wheel_speed[2] = 1;
        // ref_wheel_speed[3] = -1;

        // // just go right
        // ref_wheel_speed[0] = 1;
        // ref_wheel_speed[1] = -1;
        // ref_wheel_speed[2] = -1;
        // ref_wheel_speed[3] = 1;
    }