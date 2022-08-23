#include <iostream>
#include <unistd.h>
#include "spi2can.h"
#include "Common.h"
#include "rmd_utils.h"
#include "main.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "dynamics.h"
#include "motor_controller.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[12];
Dynamics::JMDynamics dynamics;
Motor_Controller motor_ctrl;
// FILE *Joint_Space_PD_data1;



void SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    dynamics.gain_p_joint_space[0] = msg -> data.at(0);
    dynamics.gain_p_joint_space[1] = msg -> data.at(1);
    dynamics.gain_p_joint_space[2] = msg -> data.at(2);
    // dynamics.gain_p_joint_space[3] = msg -> data.at(3);
}

void SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    dynamics.gain_d_joint_space[0] = msg -> data.at(0);
    dynamics.gain_d_joint_space[1] = msg -> data.at(1);
    dynamics.gain_d_joint_space[2] = msg -> data.at(2);
    // dynamics.gain_p_joint_space[3] = msg -> data.at(3);
}

void InitializePose(const std_msgs::BoolConstPtr &msg){
    if(msg->data) for(uint8_t i=0; i<3; i++) _DEV_MC[i].first_loop_updateTheta = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rmd_motor_controller");
    ros::Time::init();
    ros::Rate loop_rate(100);
    ros::NodeHandle node_handle_;
    ros::Publisher joint_states_pub_;
    joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("rmd/joint_states", 100);

    ros::Subscriber gain_p_sub_;
    gain_p_sub_ = node_handle_.subscribe("rmd/gain_p", 10, SwitchGainP);
    ros::Subscriber gain_d_sub_;
    gain_d_sub_ = node_handle_.subscribe("rmd/gain_d", 10, SwitchGainD);

    ros::Subscriber pose_initializer_sub_;
    pose_initializer_sub_ = node_handle_.subscribe("rmd/initialize_pose", 10, InitializePose);

    // Joint_Space_PD_data1 = fopen("/home/rainbow/catkin_ws/src/data1.dat","w");

    spi2can::getInstance();

    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));

    pthread_t thread_motion;

    int thread_id_motion = generate_rt_thread(thread_motion, rt_motion_thread, "motion_thread", 3, 97, NULL);

    while(ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<string> joints_name = {"joint1", "joint2", "joint3"};

        for (uint8_t i = 0; i<3; i ++)
        {
            msg.name.push_back(joints_name.at(i));
            msg.position.push_back(dynamics.ref_th[i]);            
            msg.position.push_back(dynamics.th[i]);
            msg.velocity.push_back(dynamics.th_dot[i]);
            msg.effort.push_back(dynamics.joint_torque[i]);            
        }
        joint_states_pub_.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void *rt_motion_thread(void *arg){
    unsigned long mpc_counter = 0;
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;
    struct timespec TIME_TOC;
    int loop_count = 0;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){
        clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        
        if(is_first_loop){
            motor_ctrl.EnableMotor();
            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            loop_count++;
        }
        else if(loop_count > 1000 ){
            dynamics.GenerateTrajectory();
            // jm_dynamics.SetTheta(motor_ctrl.GetTheta());
            dynamics.SetTheta(motor_ctrl.GetJointTheta());
            dynamics.SetThetaDot(motor_ctrl.GetThetaDot());
            dynamics.GenerateTorque_JointSpacePD();
            motor_ctrl.SetTorque(dynamics.GetTorque());
            // motor_ctrl.SetTorque(jm_dynamics.zero_vector_6);

            // fprintf(Joint_Space_PD_data1, " %lf %lf %lf %lf \n", dynamics.ref_th[0], dynamics.th[0], dynamics.ref_th[1], dynamics.th[1]);         
            // if(loop_count > 100000) fclose(Joint_Space_PD_data1);
        }
        else {
            loop_count++;
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);        

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< endl;
        }
    }
}
