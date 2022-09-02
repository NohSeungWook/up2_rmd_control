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
        int fc = 10;
        double lambda = 2*PI*fc*dt;

        for(uint8_t i=0; i<3; i++) 
        {
            th_dot[i] = lambda / (1+lambda)*theta_dot[i] + 1 / (1 +lambda) * th_dotold[i];
            th_dotold[i]= th_dot[i];
        }
        //for(uint8_t i=0; i<3; i++) th_dot[i] = theta_dot[i];
    }


    VectorXd JMDynamics::GetTorque()
    {   
        return joint_torque;
    }


    void JMDynamics::GenerateTrajectory(){
        float count_time = count * dt;
        
        count++;   

        //ee_x = 0.38 + 0.12 * sin(3.1415 / step_time * count_time);
        ee_x = 0.38 + 0.12 * cos(3.1415 / step_time * count_time);


        double l2_x = ee_x - l_3;

        cos_theta2 = (l2_x * l2_x - l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(0, l2_x) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
    }


    void JMDynamics::GenerateTorque_JointSpacePV()
    {
        if (ref_th[0]<-1.3)
        des_th[0] = -1.3;
        else if (ref_th[0]>0.7)
        des_th[0] = 0.7;
        else
        des_th[0] = ref_th[0];

        if (ref_th[1]<0)
        des_th[1] = 0;
        else if (ref_th[1]>2.5)
        des_th[1] = 2.5;
        else
        des_th[1] = ref_th[1];

        if (ref_th[2]<-1.3)
        des_th[2] = -1.3;
        else if (ref_th[2]>0)
        des_th[2] = 0;
        else
        des_th[2] = ref_th[2];

        //   gain_p_joint_space << 30,    21,      21;
        //   gain_d_joint_space << 2.7,    2.1,      2.1;
 
        double g_hat_1 = 0.739184 * cos(th[0] + th[1] + th[2]) + 6.9 * sin(th[0]-0.08) + 6.277419 * sin(th[0]) + 5.2179 * sin(th[0] + th[1]);
        double g_hat_2 = 0.739184 * cos(th[0] + th[1] + th[2]) + 5.2179 * sin(th[0] + th[1]);//last number = m2 + m3 * g
        double g_hat_3 = 0.739184 * cos(th[0] + th[1] + th[2]);//com3length * sin(angle of com3) * m3 *g

        //for (int i = 0; i < 3; i++) joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i]; 
        //for (int i = 0; i < 3; i++) joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) + gain_d_joint_space[i] * (ref_th_dot[i] - th_dot[i]); 

        // joint_torque[0] = gain_p_joint_space[0] * (ref_th[0] - th[0]) - gain_d_joint_space[0] * th_dot[0] + g_hat_1 ;
        // joint_torque[1] = gain_p_joint_space[1] * (ref_th[1] - th[1]) - gain_d_joint_space[1] * th_dot[1] + g_hat_2 ;
        // joint_torque[2] = gain_p_joint_space[2] * (ref_th[2] - th[2]) - gain_d_joint_space[2] * th_dot[2] + g_hat_3 ;
        joint_torque[0] = gain_p_joint_space[0] * (des_th[0] - th[0]) - gain_d_joint_space[0] * th_dot[0] + g_hat_1 ;
        joint_torque[1] = gain_p_joint_space[1] * (des_th[1] - th[1]) - gain_d_joint_space[1] * th_dot[1] + g_hat_2 ;
        joint_torque[2] = gain_p_joint_space[2] * (des_th[2] - th[2]) - gain_d_joint_space[2] * th_dot[2] + g_hat_3 ;


    }


    void JMDynamics::GenerateTrajectory_WalkReadyPose()
    {
        float count_time = count * dt;
        
        if(count_time < step_time )count++;   
        
        ee_x = (ux + 0.5 + (ux- 0.5) * cos(PI / step_time * count_time))/2;

        double l2_x = ee_x - l_3;

        cos_theta2 = (l2_x * l2_x - l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(0, l2_x) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
    }

    void JMDynamics::GenerateTrajectory_InitialPose()
    {
        float count_time = count * dt;
        
        if(count_time < step_time/2 )count++;   
        
        ee_x = ux + (0.6215-ux) *sin(PI / step_time * count_time);

        double l2_x = ee_x - l_3;

        cos_theta2 = (l2_x * l2_x - l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(0, l2_x) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
    }

    void JMDynamics::TrajectorySellector(int Tra_num)
    {
        if( Tra_num==1)
        GenerateTrajectory_WalkReadyPose();
        else if( Tra_num ==2)
        GenerateTrajectory();
        else
        GenerateTrajectory_InitialPose();
    }

}