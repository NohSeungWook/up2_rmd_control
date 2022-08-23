#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[3];

Motor_Controller::Motor_Controller(){
  _DEV_MC[0].Board_SetCANIDs(1, 0);   _DEV_MC[0].direction =  -1;    _DEV_MC[0].gear_ratio = 6;  _DEV_MC[0].zero_offset = 0;          _DEV_MC[0].torqueToHex = 31.25;     _DEV_MC[0].tic2rad = 10430.2197;     
  _DEV_MC[1].Board_SetCANIDs(2, 0);   _DEV_MC[1].direction =  1;    _DEV_MC[1].gear_ratio = 6;  _DEV_MC[1].zero_offset = 0;          _DEV_MC[1].torqueToHex = 31.25;     _DEV_MC[1].tic2rad = 10430.2197; 
  _DEV_MC[2].Board_SetCANIDs(3, 0);   _DEV_MC[2].direction =  -1;    _DEV_MC[2].gear_ratio = 6;  _DEV_MC[2].zero_offset = 0;          _DEV_MC[2].torqueToHex = 31.25;     _DEV_MC[2].tic2rad = 10430.2197; 
}


void Motor_Controller::EnableMotor(){
  sharedData->rmd_motor_run_flag[0] = true;
  sharedData->rmd_motor_run_flag[1] = true;
  sharedData->rmd_motor_run_flag[2] = true;

  sharedData->rmd_motor_run_flag[3] = false;  
  sharedData->rmd_motor_run_flag[4] = false;
  sharedData->rmd_motor_run_flag[5] = false;
  sharedData->rmd_motor_run_flag[6] = false;
  sharedData->rmd_motor_run_flag[7] = false;
  sharedData->rmd_motor_run_flag[8] = false;
  sharedData->rmd_motor_run_flag[9] = false;  
  sharedData->rmd_motor_run_flag[10] = false;
  sharedData->rmd_motor_run_flag[11] = false;
  
  for(uint8_t i=0; i<6; i++) {
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
  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot(){
  for(uint8_t i=0; i<3; i++) th_dot[i] = _DEV_MC[i].GetThetaDot();      
  return th_dot;
}


void Motor_Controller::SetTorque(VectorXd tau){
  for(uint8_t i=0; i<3; i++) {
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
}