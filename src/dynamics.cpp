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

        ee_x = 0.38 + 0.12 * cos(3.1415 / step_time * count_time);

        double l2_x = ee_x - l_3;

        total_loop_count++;
        if (total_loop_count == 50){
            // std::cout << "ee_x: " << ee_x << "    count_time: " << count_time << std::endl;
            total_loop_count = 1;
        }


        cos_theta2 = (l2_x * l2_x - l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(0, l2_x) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
    }


    void JMDynamics::GenerateTorque_JointSpacePD()
    {
        //   gain_p_joint_space << 30,    21,      21;
        //   gain_d_joint_space << 2.7,    2.1,      2.1;
 
        double g_hat_1 = 0.671985 * cos(th[0] + th[1] + th[2]) + 8.581788 * sin(th[0]) + 4.847121 * sin(th[0] + th[1]);
        double g_hat_2 = 0.671985 * cos(th[0] + th[1] + th[2]) + 4.847121 * sin(th[0] + th[1]);//last number = m2 + m3 * g
        double g_hat_3 = 0.671985 * cos(th[0] + th[1] + th[2]);//com3length * sin(angle of com3) * m3 *g

        //for (int i = 0; i < 3; i++) joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i]; 
        joint_torque[0] = gain_p_joint_space[0] * (ref_th[0] - th[0]) - gain_d_joint_space[0] * th_dot[0] + g_hat_1 ;
        joint_torque[1] = gain_p_joint_space[1] * (ref_th[1] - th[1]) - gain_d_joint_space[1] * th_dot[1] + g_hat_2 ;
        joint_torque[2] = gain_p_joint_space[2] * (ref_th[2] - th[2]) - gain_d_joint_space[2] * th_dot[2] + g_hat_3 ;

        //std::cout << "g_hat_1: " << g_hat_1 << "g_hat_2: " << g_hat_2 << "g_hat_3: " << g_hat_3 << std::endl;

    }


    void JMDynamics::InitialtoActivation()
    {
        float count_time = count * dt;
        
        double x1 = count_time;
        count++;   
        double x3 = pow(count_time, 3);
        double x2 = pow(count_time, 2);

        // std::cout << "x1: " << x1 << "      x2: " << x2 << "        x3: " << x3 << std::endl;
              
        
        ee_x = 0.0304 * x3 - 0.0911 * x2 + 0.6215;

        double l2_x = ee_x - l_3;

        total_loop_count++;
        if (total_loop_count == 500){
            std::cout << "      ee_x: " << ee_x << "        count_time: " << count_time << "        x2: " << x2<< std::endl;
            total_loop_count = 1;
        }


        cos_theta2 = (l2_x * l2_x - l_1 * l_1 - l_2 * l_2) / (2 * l_1 * l_2);
        sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

        phi = atan2(l_2 * sin_theta2, l_1 + l_2 * cos_theta2);

        ref_th[0] = atan2(0, l2_x) - phi;
        ref_th[1] = atan2(sin_theta2, cos_theta2);
        ref_th[2] = 0 - ref_th[0] - ref_th[1];
    }

}