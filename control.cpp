#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>
#include<tf/tf.h>
#include<geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/ParamSet.h>

#include<teknofest/velocity.h> 
#include<teknofest/arming.h> 
#include<teknofest/setmode.h> 
#include<teknofest/cv.h>

#define pi 3.14159265358979323846

//MSG TYPES
nav_msgs::Odometry odom; //infos by drone
mavros_msgs::State state; //drone's state
mavros_msgs::SetMode offb_mode; //to change drone's mode - offboard
geometry_msgs::PoseStamped pose_command; //send position
mavros_msgs::CommandBool arm_command; //make drone armed
geometry_msgs::TwistStamped velocity_command; //send velocity
mavros_msgs::ParamSet param_set; //set parameters
tf::Quaternion myQuaternion;

//CLIENTS
ros::ServiceClient arming;
ros::ServiceClient set_mode;

//SRV
teknofest::arming arm_srv;
teknofest::velocity vel_srv;
teknofest::setmode setmode_srv;
teknofest::cv cv;

//PUBLISHER
ros::Publisher velocity_pub;

bool disarm_command;

//function for getting drone's orientation in quaternion and showing in yaw
void func(const nav_msgs::Odometry::ConstPtr& msg){

    odom = *msg; 
    
    myQuaternion  = tf::Quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);      
    tf::Matrix3x3 matrix(myQuaternion);

    double roll,pitch,yaw;
    yaw=tf::getYaw(myQuaternion);  
    float pi_yaw=yaw*180/pi;
    
    ROS_INFO("yaw %f",pi_yaw); 
    
    ros::spinOnce();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

}

//state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

//arming callback - true/false
bool arm_cb(teknofest::arming::Request &req,teknofest::arming::Response &res){

    
    disarm_command = req.arming;
    

    arm_command.request.value = disarm_command;
    if(arming.call(arm_command)){//arm the drone
        if(arm_command.response.result == 0){
        ROS_INFO("Armed");
            }
    }
        
    return true;
    
}

//setmode callback -OFF_BOARD
std::string mode;

bool setmode_cb(teknofest::setmode::Request &req,teknofest::setmode::Response &res){
    mode = req.setmode;

    offb_mode.request.custom_mode = mode;
    
    if(set_mode.call(offb_mode)){//switch to offboard
        ROS_INFO("called setmode");
        if(offb_mode.response.mode_sent){
            ROS_INFO("Success");
        }
    }
    return true;
}

//velocity callback
float velocity;

bool velocity_cb(teknofest::velocity::Request &req, teknofest::velocity::Response &res){

    velocity_command = req.twistStamped;
    return true;
}

//for using infos that comes by CV
bool detected;
bool cv_cb(teknofest::cv::Request &req, teknofest::cv::Response &res){

    detected = req.detected;
    return true;
}

//publish desired position's command to drone -using PID
void go_to_position(float des_x,float des_y,float des_z){

    float error_x;
    float error_y;
    float error_z;

    float last_error_x;
    float last_error_y;
    float last_error_z;

    float current_state_x;
    float current_state_y;
    float current_state_z;

    float desired_state_x;
    float desired_state_y;
    float desired_state_z;

    float p_x;
    float p_y;
    float p_z;

    float i_x;
    float i_y;
    float i_z;

    float d_x;
    float d_y;
    float d_z;

    float Kp=1;
    float Ki=0.03;
    float Kd=0.07;

    float integral_x=0;
    float integral_y=0;
    float integral_z=0;

    float delta_time=0.1f;

    float last_velocity_x;
    float last_velocity_y;
    float last_velocity_z;

    ros::Rate rate = ros::Rate(10); 

    while(ros::ok()&& true){
        
        last_error_x=error_x;
        last_error_y=error_y;
        last_error_z=error_z;

        current_state_x=odom.pose.pose.position.x;
        current_state_y=odom.pose.pose.position.y;
        current_state_z=odom.pose.pose.position.z;
        
        desired_state_x=des_x;
        desired_state_y=des_y;
        desired_state_z=des_z;

        error_x=desired_state_x-current_state_x;
        error_y=desired_state_y-current_state_y;
        error_z=desired_state_z-current_state_z;

        integral_x+=error_x;
        integral_y+=error_y;
        integral_z+=error_z;

        if(signbit(last_error_x)!=signbit(error_x)){
            integral_x=0;
        }
        if(signbit(last_error_y)!=signbit(error_y)){
            integral_y=0;
        }
        if(signbit(last_error_z)!=signbit(error_z)){
            integral_z=0;
        }

        p_x=Kp*error_x;
        p_y=Kp*error_y;
        p_z=Kp*error_z;

        d_x=Kd*(error_x-last_error_x)/delta_time;
        d_y=Kd*(error_y-last_error_y)/delta_time;
        d_z=Kd*(error_z-last_error_z)/delta_time;

        i_x=Ki*integral_x;
        i_y=Ki*integral_y;
        i_z=Ki*integral_z;


        last_velocity_x=p_x+d_x+i_x;
        last_velocity_y=p_y+d_y+i_y;
        last_velocity_z=p_z+d_z+i_z;

        //ROS_INFO("pub velocity %f %f %f ",last_velocity_x,last_velocity_y,last_velocity_z);
        //ROS_INFO("drone velocity %f %f %f",odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
        
        
        //last velocity publishlenecek
        velocity_command.twist.linear.x=last_velocity_x;
        velocity_command.twist.linear.y=last_velocity_y;
        velocity_command.twist.linear.z=last_velocity_z;

        velocity_pub.publish(velocity_command);

        ros::Rate rate = ros::Rate(20); 
        rate.sleep();
        ros::spinOnce();

        if(error_x <1 && error_y <1 && error_z <1 ){
            break;
        
        }
    }
    

}
//publish desired yaw's command to drone -using PID ---henüz tamamlanmadı belki kullanmayız 
void go_to_yaw(float des_yaw){

    float roll=0;
    float pitch=0;
    float yaw_pi=des_yaw;//bizim gireceğimiz değer - derece
    float yaw_rad=yaw_pi*pi/180;//radyan

    float error_yaw;
    float last_error_yaw;
    float current_state_yaw;
    float desired_state_yaw;
    float p_yaw;
    float i_yaw;
    float d_yaw;
    float Kp=1;
    float Ki=0.03;
    float Kd=0.07;
    float integral_yaw=0;
    float delta_time=0.1f;
    float last_yaw;


    while(ros::ok()&& true){
        
        last_error_yaw=error_yaw;
        float pi_yaw; //geçici tanımladım
        current_state_yaw=pi_yaw;//nasıl alırız bi bak
        desired_state_yaw=des_yaw;
        error_yaw=desired_state_yaw-current_state_yaw;
        integral_yaw+=error_yaw;
        

        if(signbit(last_error_yaw)!=signbit(error_yaw)){
            integral_yaw=0;
        }
        
        p_yaw=Kp*error_yaw;
        

        d_yaw=Kd*(error_yaw-last_error_yaw)/delta_time;
        

        i_yaw=Ki*integral_yaw;
        

        last_yaw=p_yaw+d_yaw+i_yaw;
        
        yaw_rad=last_yaw*pi/180;//publishlenecek yaw
        //yawı publishle
        //buraya başka bi şey gelecek //velocity_msg.twist.linear.x=last_velocity_x;
        
        ros::Rate rate = ros::Rate(20); 
        rate.sleep();
        ros::spinOnce();

        if(error_yaw <1 ){
            break;
        }
    }
        //rad_yaw=pi_yaw*pi/180;
        //srv.request.poseStamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,rad_yaw);
        
        

}



int main(int argc, char **argv){
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    //PUBLISHER
    ros::Publisher set_point_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);

    //SUBSCRIBER
    ros::Subscriber position_listener = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,func);

    ros::Subscriber state_listener = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);

    //CLIENT
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    arming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    //SERVER
    ros::ServiceServer arm_srv = nh.advertiseService("/arm",arm_cb);

    ros::ServiceServer mode_server= nh.advertiseService("/mode",setmode_cb);

    ros::ServiceServer vel_server= nh.advertiseService("/velocity",velocity_cb);

    ros::ServiceServer cv_srv= nh.advertiseService("/cv",cv_cb);
    

    //RATE  
    ros::Rate rate = ros::Rate(20); //20Hz

    while((!state.connected) && ros::ok()){//wait until connection to mavros
        rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("%d",state.connected);

    pose_command.pose.position.x = 0;
    pose_command.pose.position.y = 0;
    pose_command.pose.position.z = 0;
    pose_command.pose.orientation.x = 0;
    pose_command.pose.orientation.y = 0;
    pose_command.pose.orientation.z = 0;
    velocity_command.twist.linear.x=0;
    velocity_command.twist.linear.y=0;
    velocity_command.twist.linear.z=0;

    for(int i = 0; (i < 40) && ros::ok(); i++){
        set_point_pos_pub.publish(pose_command);
        rate.sleep();
        ros::spinOnce();
        
    }

    offb_mode.request.custom_mode = "OFFBOARD";

    if(set_mode.call(offb_mode)){//switch to offboard
        ROS_INFO("called setmode");
        if(offb_mode.response.mode_sent){
            ROS_INFO("Success");
        }
    }

    while(ros::ok()){//main loop

        ROS_INFO("%f %f %f %d %d %s",odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,state.connected,state.armed,offb_mode.request.custom_mode.c_str());
        
        go_to_position(0,0,10);
    
        //velocity_pub.publish(velocity_command);
        
        ROS_INFO("orientation %f  ",pose_command.pose.orientation.z);
        ROS_INFO("drone orien %f  ",odom.pose.pose.orientation.z);
        ROS_INFO("velocity %f %f %f",odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
        
        
        rate.sleep();
        ros::spinOnce();
        
    }

}