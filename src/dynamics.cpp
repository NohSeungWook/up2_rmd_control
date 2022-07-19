#include "dynamics.h"

namespace Dynamics
{
    JMDynamics::JMDynamics(){}    

    void JMDynamics::SetTheta(VectorXd thetas)
    {
        for(uint8_t i=0; i<3; i++) th[i] = thetas[i];
    }


    void JMDynamics::SetThetaDot(VectorXd theta_dot)
    {
        for(uint8_t i=0; i<3; i++) th_dot[i] = theta_dot[i];
    }


    VectorXd JMDynamics::GetTorque()
    {   
        return joint_torque;
    }


    void JMDynamics::GenerateTrajectory(){
        float count_time = count * dt;     
        count++;   
        trajectory = PI * 1/2 * (1 - cos(PI * (count_time/step_time)));
    }


    void JMDynamics::GenerateTorque_JointSpacePD()
    {
        // gain_p_joint_space << 0,    0,      10;
        // gain_d_joint_space << 0,    0,      0.2;

        ref_th << 0, 0, 0;

        ref_th[0] = trajectory;
        ref_th[1] = trajectory;
        ref_th[2] = trajectory;

        for (int i = 0; i < 3; i++) joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i]; 
    }

}