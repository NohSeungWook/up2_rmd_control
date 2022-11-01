#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "shared_memory.h"
#include "dynamics.h"

#define   tic2radL   2607.435432674516
#define   tic2radX   10430.21970545193


class Motor_Controller{

public:   

  int count;
  bool first_loop = true;

  float thetas[Motor_num];  
  VectorXd theta_rad = VectorXd::Zero(Motor_num);
  VectorXd theta_deg = VectorXd::Zero(Motor_num);
  VectorXd theta_raw = VectorXd::Zero(Motor_num);
  VectorXd theta_raw_0xA1 = VectorXd::Zero(Motor_num);
  VectorXd theta_balanced_0xA1 = VectorXd::Zero(Motor_num);
  VectorXd theta_balanced_rad_0xA1 = VectorXd::Zero(Motor_num);


  VectorXd th_joint = VectorXd::Zero(Motor_num);
  VectorXd last_th_joint = VectorXd::Zero(Motor_num);
  VectorXd th_dot_joint = VectorXd::Zero(Motor_num);
  VectorXd th_motor = VectorXd::Zero(Motor_num);
  VectorXd last_th_motor = VectorXd::Zero(Motor_num);
  VectorXd th_incremental = VectorXd::Zero(Motor_num);
  VectorXd th_dot = VectorXd::Zero(Motor_num);


  Motor_Controller();
  //~Motor_Controller();

  VectorXd GetThetaX();
  VectorXd GetThetaL();
  VectorXd GetTheta();
  VectorXd GetJointTheta();
  VectorXd GetThetaDot();
  void ReadTheta();    
  void SetTorque(VectorXd tau);  
  void SetPosition(VectorXd theta);  
  void EnableMotor();
  void EmergencyMotorOff();
  void WritePIDtoROM();
};


#endif // MOTOR_CONTROLLER_H