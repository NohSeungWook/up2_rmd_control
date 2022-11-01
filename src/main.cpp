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
//FILE *Joint_Space_PD_data2;



void SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    for(uint8_t i=0; i<8; i++)
    dynamics.gain_p_joint_space[i] = msg -> data.at(i);

}
void SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    for(uint8_t i=0; i<8; i++)
    dynamics.gain_p_joint_space[i] = msg -> data.at(i);
}

//let the current angle as initial angle
void InitializePose(const std_msgs::BoolConstPtr &msg){
    if(msg->data) for(uint8_t i=0; i<8; i++) _DEV_MC[i].first_loop_updateTheta = true;
}

//write motor inner gain to ROM
void WriteToROM(const std_msgs::BoolConstPtr &msg){
    motor_ctrl.WritePIDtoROM();
}

//sellect motion
void SwitchTrajectory(const std_msgs::Int32 &msg)
{
    
    if((msg.data == 1 && dynamics.Tra_num !=1) || (msg.data == 2 && dynamics.Tra_num == 1) || (msg.data == 5 && dynamics.Tra_num != 5) || (msg.data == 3 && dynamics.Tra_num == 1))
        {
        //initialize count
        dynamics.count=0;
        //get current ee's poisiton/angle
        dynamics.ux= 0.27 * cos(dynamics.ref_th[0])+ 0.272 * cos(dynamics.ref_th[0] + dynamics.ref_th[1]) +0.0815* cos(dynamics.ref_th[0] + dynamics.ref_th[1] + dynamics.ref_th[6]);
        dynamics.ux_r = 0.27 * cos(dynamics.ref_th[2])+ 0.272 * cos(dynamics.ref_th[2] + dynamics.ref_th[3]) +0.0815* cos(dynamics.ref_th[2] + dynamics.ref_th[3] + dynamics.th[7]);
        dynamics.Tra_num =msg.data;
        dynamics.nh = dynamics.ref_th[4];
        dynamics.nh_r = dynamics.ref_th[5];
        }
    else
        dynamics.Tra_num != msg.data;
}

void SwitchGcompensation(const std_msgs::Int32 &msg){
    if (msg.data == 1)dynamics.Gfactor = msg.data;
    else if (dynamics.Gfactor_safty == 1) dynamics.Gfactor = 1;
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

    ros::Subscriber trajectory_sellector_sub_;
    trajectory_sellector_sub_ = node_handle_.subscribe("rmd/trajctory_sellect", 10, SwitchTrajectory);

    ros::Subscriber Write_PID_gain_to_ROM_sub_;
    Write_PID_gain_to_ROM_sub_ = node_handle_.subscribe("rmd/Write_PID_gain_to_ROM_sub_", 10, WriteToROM);

    ros::Subscriber On_off_G_compensation_;
    On_off_G_compensation_ = node_handle_.subscribe("rmd/On_off_G_compansation_", 10, SwitchGcompensation);

    //Joint_Space_PD_data2 = fopen("/home/rainbow/catkin_ws/src/data1.dat","w");

    spi2can::getInstance();

    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));

    pthread_t thread_motion;

    int thread_id_motion = generate_rt_thread(thread_motion, rt_motion_thread, "motion_thread", 3, 97, NULL);

    while(ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<string> joints_name = {"L hip yaw","L hip pitch", "L knee pitch", "L ankle pitch","R hip yaw", "R hip ptich", "R knee pitch", "R ankle pitch"};

        for (uint8_t i = 0; i<8; i ++)//check
        {
            msg.name.push_back(joints_name.at(i));//joint name msg
            msg.position.push_back(dynamics.ref_th[i]);//reference angle msg
            msg.position.push_back(dynamics.th[i]);//actual angle msg
            msg.velocity.push_back(dynamics.th_dot[i]);// actual angle speed
            msg.effort.push_back(dynamics.joint_torque[i]);//reference torque msg        
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
            dynamics.TrajectorySellector(dynamics.Tra_num);           
            dynamics.SetTheta(motor_ctrl.GetJointTheta());
            dynamics.SetThetaDot(motor_ctrl.GetThetaDot());
            dynamics.GenerateTorque_JointSpacePV(dynamics.Gfactor);
            motor_ctrl.SetTorque(dynamics.GetTorque());
            // if (dynamics.th[0]<-1.4 || dynamics.th[0]>0.7 || dynamics.th[1]<-0.1 || dynamics.th[1] > 2.5 || dynamics.th[2]<-1.43 || dynamics.th[2]>0 || \
            // dynamics.th[3]<-1.4 || dynamics.th[3]>0.7 || dynamics.th[4]<-0.1 || dynamics.th[4] > 2.5 || dynamics.th[5]<-1.43 || dynamics.th[5]>0)
            // motor_ctrl.EmergencyMotorOff();

            //fprintf(Joint_Space_PD_data2, " %lf %lf %lf %lf %lf %lf \n", dynamics.th[0], dynamics.th[1], dynamics.th[2],\
            dynamics.th[3], dynamics.th[4], dynamics.th[5]);         
            //if(loop_count > 100000) fclose(Joint_Space_PD_data2);
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
