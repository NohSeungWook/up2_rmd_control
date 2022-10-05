#include "dynamics.h"
#include "motor_controller.h"

namespace Dynamics
{
    JMDynamics::JMDynamics(){}    

    void JMDynamics::SetTheta(VectorXd thetas)
    {
        for(uint8_t i=0; i<6; i++) th[i] = thetas[i];
        //if (th[0]<-1.4 || th[0]>0.7 || th[1]<-0.1 || th[1] > 2.5 || th[2]<-.13 || th[2]>0)
        //Motor_Controller.EmergencyMotorOff();
    }


    void JMDynamics::SetThetaDot(VectorXd theta_dot)
    {
        // int fc = 10;
        // double lambda = 2*PI*fc*dt;

        // for(uint8_t i=0; i<3; i++) 
        // {
        //     th_dot[i] = lambda / (1+lambda)*theta_dot[i] + 1 / (1 +lambda) * th_dotold[i];
        //     th_dotold[i]= th_dot[i];
        // }
        for(uint8_t i=0; i<6; i++) th_dot[i] = theta_dot[i];
    }


    VectorXd JMDynamics::GetTorque()
    {   
        return joint_torque;
    }

//trajectory for non torque situation
void JMDynamics::GenerateNonTrajectory(){
        float count_time = count * dt;
        
        count++;   

        ref_th[0] = th[0];
        ref_th[1] = th[1];
        ref_th[2] = th[2];

        ref_th[3] = th[3];
        ref_th[4] = th[4];
        ref_th[5] = th[5];
    }

     void JMDynamics::SolveIK(double ee_x, double ee_y, double ee_th)
    {
        double s = ee_x - l_3 * cos(ee_th);
        double r = ee_y- l_3 * sin(ee_th);

        cos_theta2 = (s * s + r * r- l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(r, s) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
        ref_th[3] = atan2(r, s) - phi;
        ref_th[4] = atan2(sin_theta2, cos_theta2);
        ref_th[5] = 0 - ref_th[3] - ref_th[4];
    }


    void JMDynamics::GenerateTrajectory(){
        float count_time = count * dt;
        
        count++;   

        //ee_x = 0.38 + 0.12 * sin(3.1415 / step_time * count_time);
        ee_x = 0.38 + 0.12 * cos(3.1415 / step_time * count_time);
        ee_y = 0;
        ee_th = 0;

        SolveIK(ee_x,ee_y, ee_th);
    }


    void JMDynamics::GenerateTorque_JointSpacePV()
    {
        //   gain_p_joint_space << 30,    21,      21;
        //   gain_d_joint_space << 2.7,    2.1,      2.1;
 
        //double g_hat_1 = 0.5072 * cos(th[0] + th[1] + th[2]) + 4.1610 * sin(th[0] + th[1]) + 5.8215* sin(th[0]-0.08) + 5.6417* sin(th[0]);
        // double g_hat_2 = 0.5072 * cos(th[0] + th[1] + th[2]) + 4.1610 * sin(th[0] + th[1]);//last number = m2 + m3 * g
        // double g_hat_3 = 0.5072 * cos(th[0] + th[1] + th[2]);//com3length * sin(angle of com3) * m3 *g

        for (int i = 0; i < 6; i++) joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i]; 
  
        // joint_torque[0] = gain_p_joint_space[0] * (ref_th[0] - th[0]) - gain_d_joint_space[0] * th_dot[0]; //+ g_hat_1 ;
        // joint_torque[1] = gain_p_joint_space[1] * (ref_th[1] - th[1]) - gain_d_joint_space[1] * th_dot[1];// + g_hat_2 ;
        // joint_torque[2] = gain_p_joint_space[2] * (ref_th[2] - th[2]) - gain_d_joint_space[2] * th_dot[2];// + g_hat_3 ;
    }


    void JMDynamics::GenerateTrajectory_WalkReadyPose()
    {
        float count_time = count * dt;
        
        if(count_time < step_time )count++;   
        
        ee_x = (ux + 0.5 + (ux- 0.5) * cos(PI / step_time * count_time))/2;
        ee_y = 0;
        ee_th = 0;

        SolveIK(ee_x,0,0);
    }

    void JMDynamics::GenerateTrajectory_InitialPose()
    {
        float count_time = count * dt;
        
        if(count_time < step_time/2 )count++;   
        
        ee_x = ux + (0.6255-ux) *sin(PI / step_time * count_time);

        SolveIK(ee_x, 0 ,0);
    }

     void JMDynamics::GenerateTrajectory_UptoDown()
    {
        float count_time = count * dt;
        
        if(count_time < step_time/2 )count++;   
        
        ee_x = ux + (0.3831-ux) *sin(PI / step_time * count_time);

        SolveIK(ee_x,0,0);
    }
    

    void JMDynamics::TrajectorySellector(int Tra_num)
    {
        if( Tra_num==5)
        GenerateTrajectory_InitialPose();
        if( Tra_num==1)
        GenerateTrajectory_WalkReadyPose();
        else if( Tra_num ==2)
        GenerateTrajectory();
        else if( Tra_num ==3)
        GenerateTrajectory_UptoDown();
        else
        GenerateNonTrajectory();
    }

}