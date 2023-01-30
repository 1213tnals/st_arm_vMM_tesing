#include "callback.h"

extern rmd_motor _BASE_MC[4];
extern Dynamics::JMDynamics jm_dynamics;
extern Mobile_Base base_ctrl;


Callback::Callback() {}


void Callback::SwitchMode(const std_msgs::Int32ConstPtr &msg)
{
  jm_dynamics.SwitchMode(msg);
}

void Callback::HMDTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  jm_dynamics.hmd_position.x() = msg->pose.position.x;
  jm_dynamics.hmd_position.y() = msg->pose.position.y;
  jm_dynamics.hmd_position.z() = msg->pose.position.z;

  jm_dynamics.hmd_quaternion.x() = msg->pose.orientation.x;
  jm_dynamics.hmd_quaternion.y() = msg->pose.orientation.y;
  jm_dynamics.hmd_quaternion.z() = msg->pose.orientation.z;
  jm_dynamics.hmd_quaternion.w() = msg->pose.orientation.w;
}

void Callback::SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  jm_dynamics.gain_p_task_space[0] = msg->data.at(0);
  jm_dynamics.gain_p_task_space[1] = msg->data.at(1);
  jm_dynamics.gain_p_task_space[2] = msg->data.at(2);
}

void Callback::SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  jm_dynamics.gain_w_task_space[0] = msg->data.at(0);
  jm_dynamics.gain_w_task_space[1] = msg->data.at(1);
  jm_dynamics.gain_w_task_space[2] = msg->data.at(2);
}

void Callback::SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 7; i++) jm_dynamics.gain_p_joint_space[i] = msg -> data.at(i);
}

void Callback::SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 7; i++) jm_dynamics.gain_d_joint_space[i] = msg -> data.at(i);
}

void Callback::SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 6; i++) jm_dynamics.gain_r[i] = msg -> data.at(i);
}

void Callback::InitializePose(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data) for(uint8_t i=0; i<6; i++) _BASE_MC[i].initialize_position = true;
  std::cout << "Initialized Pose" << std::endl;
}

void Callback::GripperCallback(const std_msgs::Float32ConstPtr &msg)
{
  jm_dynamics.SetGripperValue(msg->data);
}

void Callback::JoysticCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  base_ctrl.joystick_position.x() = msg->pose.position.x;
  base_ctrl.joystick_position.y() = msg->pose.position.y;
  base_ctrl.joystick_position.z() = msg->pose.position.z;

  base_ctrl.joystick_quaternion.x() = msg->pose.orientation.x;
  base_ctrl.joystick_quaternion.y() = msg->pose.orientation.y;
  base_ctrl.joystick_quaternion.z() = msg->pose.orientation.z;
  base_ctrl.joystick_quaternion.w() = msg->pose.orientation.w;
}