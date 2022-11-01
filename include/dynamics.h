#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Common.h"

#include <eigen3/Eigen/Dense>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;


#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define g           9.81;       

#define inner_dt 0.001

#define Motor_num 12



namespace Dynamics
{
    class JMDynamics
    {
        double dt = 0.002;
        double time = 0;

        double cnt_time = 0;
        unsigned int cnt = 0;   
        

    public:
        JMDynamics();
        //~JMDynamics();

        VectorXd ref_th = VectorXd::Zero(Motor_num);
        VectorXd des_th = VectorXd::Zero(Motor_num);
        VectorXd th = VectorXd::Zero(Motor_num);
        VectorXd th_dot = VectorXd::Zero(Motor_num);
        VectorXd th_dotold = VectorXd::Zero(Motor_num);
        VectorXd zero_vector_3 = VectorXd::Zero(Motor_num);

        int count = 0;
        int step_time = 4;
        

        double Ex = 0;
        double ux = 0;
        double ux_r = 0;
        double nh = 0;
        double nh_r = 0;
      


        double ee_x, ee_xr, ee_y, ee_z, ee_th ,phi;

        int Tra_num = 0;
        int Gfactor = 0;
        int Gfactor_safty = 0;

        double cos_theta2, sin_theta2;

        double l_1 = 0.27;
        double l_2 = 0.272;
        double l_3 = 0.0815;
        double cl_1 = 0.135;
        double cl_2 = 0.135;
        double cl_3 = 0.05;

        VectorXd joint_torque = VectorXd::Zero(Motor_num);
        VectorXd ref_joint_torque = VectorXd::Zero(Motor_num);

        VectorXd gain_p_joint_space = VectorXd::Zero(Motor_num);
        VectorXd gain_d_joint_space = VectorXd::Zero(Motor_num);

        VectorXd GetTorque();
        void SetTheta(VectorXd);
        void SetThetaDot(VectorXd);
        void GenerateTorque_JointSpacePV(int i);
        void GenerateTrajectory();
        void SolveIK(double x, double y, double th, int jn1, int jn2, int jn3);
        void GenerateNonTrajectory();
        void GenerateTrajectory_InitialPose();
        void GenerateTrajectory_WalkReadyPose();
        void GenerateTrajectory_UptoDown();
    
        void TrajectorySellector(int i);
    };
}


#endif // DYNAMICS_H