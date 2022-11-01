#include "motor_controller.h"
#include "rmd_motor.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[12];

Motor_Controller::Motor_Controller(){
  //boar_setcanids(#canidnumber,spichannumber), direction(revolving direction), zero_offset(initial degree), torqueToHex(62.5/torqueconstant), tic2rad(change_encordervalue to rad)
  _DEV_MC[0].Board_SetCANIDs(1, 0);   _DEV_MC[0].direction =  -1;    _DEV_MC[0].gear_ratio = 9;  _DEV_MC[0].zero_offset = 0;          _DEV_MC[0].torqueToHex = 29.9043;     _DEV_MC[0].tic2rad = tic2radL; //left hip pitch
  _DEV_MC[1].Board_SetCANIDs(2, 0);   _DEV_MC[1].direction =  1;    _DEV_MC[1].gear_ratio = 36;  _DEV_MC[1].zero_offset = 0;          _DEV_MC[1].torqueToHex = 9.0318;     _DEV_MC[1].tic2rad = tic2radL; //left knee pitch
  _DEV_MC[2].Board_SetCANIDs(3, 1);   _DEV_MC[2].direction =  1;    _DEV_MC[2].gear_ratio = 9;  _DEV_MC[2].zero_offset = 0;          _DEV_MC[2].torqueToHex = 29.9043;     _DEV_MC[2].tic2rad = tic2radL; //right hip pitch
  _DEV_MC[3].Board_SetCANIDs(4, 1);   _DEV_MC[3].direction =  -1;    _DEV_MC[3].gear_ratio = 36;  _DEV_MC[3].zero_offset = 0;          _DEV_MC[3].torqueToHex = 9.0318;     _DEV_MC[3].tic2rad = tic2radL; //right knee pitch
  _DEV_MC[4].Board_SetCANIDs(1, 2);   _DEV_MC[4].direction =  -1;    _DEV_MC[4].gear_ratio = 6;  _DEV_MC[4].zero_offset = 0;          _DEV_MC[4].torqueToHex = 71.0227;     _DEV_MC[4].tic2rad = tic2radL; //left hip yaw
  _DEV_MC[5].Board_SetCANIDs(2, 2);   _DEV_MC[5].direction =  1;    _DEV_MC[5].gear_ratio = 6;  _DEV_MC[5].zero_offset = 0;          _DEV_MC[5].torqueToHex = 71.0227;     _DEV_MC[5].tic2rad = tic2radL; //right hip yaw
  _DEV_MC[6].Board_SetCANIDs(3, 3);   _DEV_MC[6].direction =  -1;    _DEV_MC[6].gear_ratio = 6;  _DEV_MC[6].zero_offset = 0;          _DEV_MC[6].torqueToHex = 31.25;     _DEV_MC[6].tic2rad = tic2radX; //left ankle pitch
  _DEV_MC[7].Board_SetCANIDs(4, 3);   _DEV_MC[7].direction =  1;    _DEV_MC[7].gear_ratio = 6;  _DEV_MC[7].zero_offset = 0;          _DEV_MC[7].torqueToHex = 31.25;     _DEV_MC[7].tic2rad = tic2radX; //right ankle pitch

  // _DEV_MC[0].direction =  1;    _DEV_MC[0].gear_ratio = 9;    _DEV_MC[0].zero_offset = 0;          _DEV_MC[0].torqueToHex = 30;         _DEV_MC[0].tic2rad = tic2radL;
  // _DEV_MC[1].direction =  1;    _DEV_MC[1].gear_ratio = 36;   _DEV_MC[1].zero_offset = 0;          _DEV_MC[1].torqueToHex = 9.0318;     _DEV_MC[1].tic2rad = tic2radL;
  // _DEV_MC[2].direction =  1;    _DEV_MC[2].gear_ratio = 9;    _DEV_MC[2].zero_offset = 0;          _DEV_MC[2].torqueToHex = 30;         _DEV_MC[2].tic2rad = tic2radL;


}

//motor activation control
void Motor_Controller::EnableMotor()
{
  sharedData->rmd_motor_run_flag[0] = true;
  sharedData->rmd_motor_run_flag[1] = true;
  sharedData->rmd_motor_run_flag[2] = true;
  sharedData->rmd_motor_run_flag[3] = true;  
  sharedData->rmd_motor_run_flag[4] = false;
  sharedData->rmd_motor_run_flag[5] = false;

  sharedData->rmd_motor_run_flag[6] = true;
  sharedData->rmd_motor_run_flag[7] = true;
  sharedData->rmd_motor_run_flag[8] = true;
  sharedData->rmd_motor_run_flag[9] = true;  
  sharedData->rmd_motor_run_flag[10] = false;
  sharedData->rmd_motor_run_flag[11] = false;
  
  for(uint8_t i=0; i<8; i++) 
  {
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
  for(uint8_t i=0; i<8; i++) th_joint[i] = _DEV_MC[i].GetTheta();      
  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot(){
  for(uint8_t i=0; i<8; i++) th_dot[i] = _DEV_MC[i].GetThetaDot();      
  return th_dot;
}

//torque control command
void Motor_Controller::SetTorque(VectorXd tau){
  for(uint8_t i=0; i<8; i++) {
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

//Motor emergency off command
void Motor_Controller::EmergencyMotorOff(){
  for(uint8_t i=0; i<8; i++) {
    
    _DEV_MC[i].ref_data[0] = 0x80 & 0xFF;
    _DEV_MC[i].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[2] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[3] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[4] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[5] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[6] = 0x00 & 0xFF;
    _DEV_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}

//Motor inner gain control command
void Motor_Controller::WritePIDtoROM(){    
    _DEV_MC[0].ref_data[0] = 0x32 & 0xFF;
    _DEV_MC[0].ref_data[1] = 0x00 & 0xFF;
    _DEV_MC[0].ref_data[2] = 50 & 0xFF;
    _DEV_MC[0].ref_data[3] = 40 & 0xFF;
    _DEV_MC[0].ref_data[4] = 50 & 0xFF;
    _DEV_MC[0].ref_data[5] = 30 & 0xFF;
    _DEV_MC[0].ref_data[6] = 50 & 0xFF;
    _DEV_MC[0].ref_data[7] = 40 & 0xFF;

}

// 0x141 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00
// void Motor_Controller::EnableFilter(){
//   for(uint8_t i=0; i<3; i++) {
//     _DEV_MC[i].ref_data2[0] = 0x20 & 0xFF;
//     _DEV_MC[i].ref_data2[1] = 0x02 & 0xFF;
//     _DEV_MC[i].ref_data2[2] = 0x00 & 0xFF;
//     _DEV_MC[i].ref_data2[3] = 0x00 & 0xFF;
//     _DEV_MC[i].ref_data2[4] = 0x01 & 0xFF;
//     _DEV_MC[i].ref_data2[5] = 0x00 & 0xFF;
//     _DEV_MC[i].ref_data2[6] = 0x00 & 0xFF;
//     _DEV_MC[i].ref_data2[7] = 0x00 & 0xFF;
//   }
// }