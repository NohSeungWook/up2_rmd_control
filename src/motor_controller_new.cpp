#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[7];
extern Dynamixel _WRIST_MC;

Motor_Controller::Motor_Controller(){

  // _DEV_MC[0].Board_SetCANIDs(0, 0);   _DEV_MC[0].direction = -1;    _DEV_MC[0].gear_ratio = 8;  _DEV_MC[0].zero_offset = 0.0;       _DEV_MC[0].torqueToHex = 71.02;  _DEV_MC[0].torque_limit = 3.5*2;     _DEV_MC[0].tic2rad = tic2radL; // 10430.2197
  _DEV_MC[0].Board_SetCANIDs(0, 0);   _DEV_MC[0].direction = -1;    _DEV_MC[0].gear_ratio = 8;  _DEV_MC[0].zero_offset = 0.0;       _DEV_MC[0].torqueToHex = 200;  _DEV_MC[0].torque_limit = 4.5*2;     _DEV_MC[0].tic2rad = tic2radL; // 10430.2197
  _DEV_MC[1].Board_SetCANIDs(1, 1);   _DEV_MC[1].direction =  1;    _DEV_MC[1].gear_ratio = 8;  _DEV_MC[1].zero_offset = -PI;     _DEV_MC[1].torqueToHex = 200;    _DEV_MC[1].torque_limit = 4.5*2;     _DEV_MC[1].tic2rad = tic2radL;   //53.42
  _DEV_MC[2].Board_SetCANIDs(2, 2);   _DEV_MC[2].direction = -1;    _DEV_MC[2].gear_ratio = 8;  _DEV_MC[2].zero_offset = 2.8992;      _DEV_MC[2].torqueToHex = 200;    _DEV_MC[2].torque_limit = 4.5*2;     _DEV_MC[2].tic2rad = tic2radL;
  // _DEV_MC[3].Board_SetCANIDs(6, 3);   _DEV_MC[3].direction = -1;    _DEV_MC[3].gear_ratio = 1;  _DEV_MC[3].zero_offset = 0.3;      _DEV_MC[3].torqueToHex = 272;    _DEV_MC[3].torque_limit = 0.36*2;    _DEV_MC[3].tic2rad = tic2radL;
  // _DEV_MC[4].Board_SetCANIDs(7, 3);   _DEV_MC[4].direction = -1;    _DEV_MC[4].gear_ratio = 1;  _DEV_MC[4].zero_offset = 0;         _DEV_MC[4].torqueToHex = 625;    _DEV_MC[4].torque_limit = 0.15*2;    _DEV_MC[4].tic2rad = tic2radL;
  // _DEV_MC[5].Board_SetCANIDs(8, 3);   _DEV_MC[5].direction = -1;    _DEV_MC[5].gear_ratio = 1;  _DEV_MC[5].zero_offset = 0;         _DEV_MC[5].torqueToHex = 625;    _DEV_MC[5].torque_limit = 0.15*2;    _DEV_MC[5].tic2rad = tic2radL; // 521

}

void Motor_Controller::EnableMotor(){
  sharedData->rmd_motor_run_flag[0] = true;  // 0xA1
  sharedData->rmd_motor_run_flag[1] = false;   // 0x92

  sharedData->rmd_motor_run_flag[2] = true;
  sharedData->rmd_motor_run_flag[3] = false;
  
  sharedData->rmd_motor_run_flag[4] = false;
  sharedData->rmd_motor_run_flag[5] = false;

  sharedData->rmd_motor_run_flag[6] = true;
  sharedData->rmd_motor_run_flag[7] = false;

  sharedData->rmd_motor_run_flag[8] = false;
  sharedData->rmd_motor_run_flag[9] = false;  
  sharedData->rmd_motor_run_flag[10] = false;

  sharedData->rmd_motor_run_flag[11] = false;
  
  for(uint8_t i=0; i<3; i++) {
    _DEV_MC[i].first_loop_updateTheta = true;
    _DEV_MC[i].ref_data[0] = 0x88 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}


VectorXd Motor_Controller::GetJointTheta(){
  for(uint8_t i=0; i<3; i++) th_joint[i] = _DEV_MC[i].GetTheta();
  VectorXd th_(4); th_ = _WRIST_MC.GetThetaAct(); for(uint8_t i=0; i<4; i++) th_joint[i+3] = th_[i];
  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot(){
  for(uint8_t i=0; i<3; i++) th_dot[i] = _DEV_MC[i].GetThetaDot();
  VectorXd th_dot_(4); th_dot_ = _WRIST_MC.GetThetaDot(); for(uint8_t i=0; i<4; i++) th_dot[i+3] = th_dot_[i];
  // th_dot[0] = th_dot[0] / 6;
  return th_dot;
}


VectorXd Motor_Controller::GetThetaDotEst(){
  VectorXd th_dot_est_(4); th_dot_est_ = _WRIST_MC.GetThetaDotEstimated();
  return th_dot_est_;
}

// Simple Moving Average filtered Joint Velocity
VectorXd Motor_Controller::GetThetaDotSMAF(){
  for(uint8_t i=0; i<3; i++) th_dot[i] = _DEV_MC[i].GetThetaDot();    // Get from shoulder motors RMD-X6
  // VectorXd th_dot_(4); th_dot_ = _WRIST_MC.GetThetaDot(); for(uint8_t i=0; i<4; i++) th_dot[i+3] = th_dot_[i];   // Get from wrist motors: Dynamixel from motor
  VectorXd a_th_dot(4); a_th_dot = _WRIST_MC.GetThetaDotEstimated(); for(uint8_t i=0; i<4; i++) th_dot[i+3] = a_th_dot[i];   // Get from wrist motors: Dynamixel from estimated
  // th_dot[0] = th_dot[0] / 6; // Because V2 motor driver

  sma << sma.block<window_size-1, 7>(1, 0), th_dot[0], th_dot[1], th_dot[2], th_dot[3], th_dot[4], th_dot[5], th_dot[6];
  th_dot_sma_filtered = sma.colwise().mean();

  return th_dot_sma_filtered;
}


void Motor_Controller::SetTorque(VectorXd tau){
  for(uint8_t i=0; i<3; i++) {
    if (true) {
      if (tau[i] > _DEV_MC[i].torque_limit) tau[i] = _DEV_MC[i].torque_limit;
      if (tau[i] < -1 * _DEV_MC[i].torque_limit) tau[i] = -1 * _DEV_MC[i].torque_limit;
    }
    long param = _DEV_MC[i].direction * _DEV_MC[i].torqueToHex * tau[i];
    _DEV_MC[i].ref_data[0] = 0xa1 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = (param     ) & 0xFF;
    _DEV_MC[i].ref_data[5] = (param >> 8) & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }

  for(uint8_t i=0; i<4; i++) torque_wrist[i] = tau[i+3];
  _WRIST_MC.SetTorqueRef(torque_wrist);
}


void Motor_Controller::ReadTheta(){
  for(uint8_t i=0; i<3; i++) {
    _DEV_MC[i].ref_data2[0] = 0x92 & 0xFF;
    _DEV_MC[i].ref_data2[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[4] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[7] = 0x00 & 0xFF;
  }
}

// 0x141 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00
void Motor_Controller::EnableFilter(){
  for(uint8_t i=0; i<3; i++) {
    _DEV_MC[i].ref_data2[0] = 0x20 & 0xFF;
    _DEV_MC[i].ref_data2[1] = 0x02 & 0xFF;
    _DEV_MC[i].ref_data2[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[4] = 0x01 & 0xFF;
    _DEV_MC[i].ref_data2[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data2[7] = 0x00 & 0xFF;
  }
}