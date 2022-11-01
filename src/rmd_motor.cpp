#include "rmd_motor.h"

rmd_motor::rmd_motor()
{
    MoveJoints.MoveFlag = false;
    CurrentStatus.B[0] = CurrentStatus.B[1] = CurrentStatus.B[2] = 0;
    homeManualOffset = 0;
}

void rmd_motor::Board_GetEncData(void) {
    if(enc_data[1] == 0) {
        int temp_enc = (int)(enc_data[4] | (enc_data[5]<<8) | (enc_data[6]<<16) | (enc_data[7]<<24));  
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
        MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
    }
    else {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | ((enc_data[3]&0xF0)<<12));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
        MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
    }
}

void rmd_motor::Board_GetEncData2(void) {
    if(enc_data[7] == 0) {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | (enc_data[3]<<16) | (enc_data[4]<<24));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
    }
    else {
        int temp_enc = (int)(enc_data[1] | (enc_data[2]<<8) | ((enc_data[3]&0xF0)<<12));
        if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
        EncoderValue = temp_enc;
    }    
    MeasuredPosition_deg = (float)EncoderValue/PPR + zeroManualOffset;
}

void rmd_motor::Board_SetTorqueDataX(void) {
    //int temp_torque = (int)(torque_data[2] | (torque_data[3]<<8));
    //_torque_ctrl_torque_fdback = temp_torque;
    int temp_speed = (int)(torque_data[4] | (torque_data[5]<<8));
    if(temp_speed & 0x80000) temp_speed |= 0xFFF00000;
    if(temp_speed > 30000) temp_speed -= 65535;
    //  _torque_ctrl_speed_fdback = 0.01 * temp_speed / gear_ratio * direction;
    _torque_ctrl_speed_fdback = 0.01 * temp_speed * direction;
    
    // _torque_ctrl_encoder_fdback = temp_enc;

    int temp_enc = (int)(torque_data[6] | (torque_data[7]<<8));
    // if(temp_enc & 0x80000) temp_enc |= 0xFFF00000;
    th_motor = temp_enc / tic2rad;
    if(first_loop_updateTheta){
        th_joint = zero_offset;
        last_th_motor = th_motor;
        first_loop_updateTheta = false;
        th_incremental = 0;
    }
    else{
        th_incremental = th_motor - last_th_motor;
        last_th_motor = th_motor;
    }
    
    if (th_incremental > 4) th_incremental -= 6.28319;
    else if (th_incremental < -4) th_incremental += 6.28319;
    th_joint += th_incremental / gear_ratio * direction;
}

void rmd_motor::UpdateTheta(void) {
    th_motor = _torque_ctrl_encoder_fdback / tic2rad;
    if(first_loop_updateTheta){
        th_joint = zero_offset;
        last_th_motor = th_motor;
        first_loop_updateTheta = false;
    }
    th_incremental = th_motor - last_th_motor;
    last_th_motor = th_motor;
    if (th_incremental > 4) th_incremental -= 6.28319;
    else if (th_incremental < -4) th_incremental += 6.28319;
    th_joint += th_incremental / gear_ratio * direction;
}

float rmd_motor::GetTheta() {
    return th_joint;
}

float rmd_motor::GetThetaDot() {
    return _torque_ctrl_speed_fdback;
}

// HRRLAB - CAN ID SETTING
void rmd_motor::Board_SetCANIDs(int bno, int can_ch){
    BOARD_ID = bno;
    CAN_CHANNEL = can_ch;
    ID_RMD_COMMAND = 0x140+bno;
}
