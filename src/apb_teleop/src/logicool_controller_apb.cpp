#include "ros/ros.h"
#include "apb_teleop/logicool_button_mapping.h"
#include "sensor_msgs/Joy.h"
#include "apb_msgs/FanDC.h"
#include "apb_msgs/MotorPos.h"
#include "apb_msgs/MotorDCval.h"
#include "apb_msgs/MotorSpeed.h"
#include "std_msgs/Int64.h"


bool joy_switch_state = false;

double current_axes_factor[4];

int motor_speed[3]={0,0,0};
bool motor_speed_switch = false;  

int motor_pos[3]={0,0,0};
bool motor_pos_switch = false;  

int motor_dcval[3]={1500,1500,1500};
bool motor_dcval_switch = false; 


int fan_dcval[3]={0,0,0};
bool fan_dcval_switch = false; 

bool fan_1_dcval_switch = false;
bool fan_2_dcval_switch = false;
bool fan_3_dcval_switch = false;
//bool fan_4_dcval_switch = false;



void JoyCallback(const sensor_msgs::Joy msg){
    //Increase/decrease 10.0 eachtime
    //速度范围 -50~50 mm/s
    if(1 == msg.axes[AXES_CROSS_UD]){
        if(motor_speed[1] >= 50){
            std::cout<<"Warming!!! The motor speed can't be faster!!!";
            for(int i = 0; i < 3; i++){
                motor_speed[i] = 0.0;
            }
        }
        else{
            for(int i = 0; i < 3; i++){ 
                motor_speed[i] += 10.0;
            }
            std::cout<<"The current motor_speed is "<<motor_speed[1]<<"!!!"<<std::endl;
            motor_speed_switch = true; 
        }
    }
    else if(-1 == msg.axes[AXES_CROSS_UD]){
       if(motor_speed[1] <= -50){
            std::cout<<"Warming!!! The motor speed can't be smaller!!!";
            for(int i = 0; i < 3; i++){
                motor_speed[i] = 0.0;
            }
        }
        else{
            for(int i = 0; i < 3; i++){
                motor_speed[i] -= 10.0;
            }
            std::cout<<"The current motor_speed is "<<motor_speed[1]<<"!!!"<<std::endl;
            motor_speed_switch = true; 
        }
    }
    else if(1 == msg.buttons[BUTTON_SHAPE_LB]){
            for(int i = 0; i < 3; i++){
                motor_speed[i] = 0.0;
            }
            std::cout<<"The current motor is stopped !!! "<<std::endl;
            motor_speed_switch = true;
    }


    
/*舵轮转向
    if(1 == msg.axes[AXES_CROSS_LR]){
        if(motor_pos[0] >= 90){
            std::cout<<"Warming!!! The motor angle can't be bigger!!!";
            for(int i = 0; i < 3; i++){
                motor_pos[i] = 0.0;
            }
        }
        else{
            for(int i = 0; i < 3; i++){
                motor_pos[i] -= 10.0;
            }
            std::cout<<"The current motor_position is "<<motor_pos[0]<<"!!!"<<std::endl;
            motor_pos_switch = true; 
        }
    }
    else if(-1 == msg.axes[AXES_CROSS_LR]){
        if(motor_pos[0] <= -90){
            std::cout<<"Warming!!! The motor angle can't be smaller!!!";
            for(int i = 0; i < 3; i++){
                motor_pos[i] = 0.0;
            }
        }
        else{
            for(int i = 0; i < 3; i++){
                motor_pos[i] += 10.0;
            }
            std::cout<<"The current motor_position is "<<motor_pos[0]<<"!!!"<<std::endl;
            motor_pos_switch = true; 
        }
    }
    else if(1 == msg.buttons[BUTTON_LT]){
        for(int i = 0; i < 3; i++){
            motor_pos[i] = 0.0;
        }
        std::cout<<"The current motor_position is 0 !!!"<<std::endl;
        motor_pos_switch = true; 
    }
    */
   
    //转向 0原位 1右转 2左转
    if(1 == msg.axes[AXES_CROSS_LR]){
        for(int i = 0; i < 3; i++){
                motor_pos[i] = 1.0;
            }
        std::cout<<"The current motor_position is  turning right"<<"!!!"<<std::endl;
        motor_pos_switch = true; 
       
    }
    else if(-1 == msg.axes[AXES_CROSS_LR]){
        for(int i = 0; i < 3; i++){
                motor_pos[i] = 2;
            }
        std::cout<<"The current motor_position is  turning left"<<"!!!"<<std::endl;
        motor_pos_switch = true; 
    }
    else if(1 == msg.buttons[BUTTON_LT]){
        for(int i = 0; i < 3; i++){
            motor_pos[i] = 0.0;
        }
        std::cout<<"The current motor_position is original_position !!!"<<std::endl;
        motor_pos_switch = true; 
    }


    //水下电机范围 1250~1750
    if(1 == msg.buttons[BUTTON_SHAPE_STICK_LEFT]){     
        if(motor_dcval[0] >= 1750){
            std::cout<<"Warming!!! The motor_dcval can't be bigger!!!";
            for(int i = 0; i < 3; i++){
                motor_dcval[i] = 1500.0;    //停止转动
            }
        }
        else{
            for(int i = 0; i < 3; i++){
                motor_dcval[i] += 50.0;
            }
            std::cout<<"The current motor_position is "<<motor_dcval[0]<<"!!!"<<std::endl;
            motor_dcval_switch = true; 
        }
    }
    else if(1 == msg.buttons[BUTTON_SHAPE_STICK_RIGHT]){
        if(motor_dcval[0] <= 1250){
            std::cout<<"Warming!!! The motor_dcval can't be smaller!!!";
            for(int i = 0; i < 3; i++){
                motor_dcval[i] = 1500.0;    
            }
        }
        else{
            for(int i = 0; i < 3; i++){
                motor_dcval[i] -= 50.0;
            }
            std::cout<<"The current motor_position is "<<motor_dcval[0]<<"!!!"<<std::endl;
            motor_dcval_switch = true; 
        }
    }
    else if(1 == msg.buttons[BUTTON_SHAPE_RB]){
        for(int i = 0; i < 3; i++){
                motor_dcval[i] = 1500.0;    
        }
        std::cout<<"The current motor_dcval has been reset to 1500 !!!"<<std::endl;
        motor_dcval_switch = true; 
    }


    //风机 0~1000
    if(1 == msg.buttons[BUTTON_SHAPE_Y]){            
        fan_1_dcval_switch = ! fan_1_dcval_switch;
        fan_dcval[0] = (240.0 * fan_1_dcval_switch);
        std::cout<<"The current fan_1_dcval is "<<fan_dcval[0]<<"!!!"<<std::endl;
        fan_dcval_switch = true;
    }
    else if(1 == msg.buttons[BUTTON_SHAPE_X]){     

        fan_2_dcval_switch = ! fan_2_dcval_switch;
        fan_dcval[1] = (240.0 * fan_2_dcval_switch);
        std::cout<<"The current fan_2_dcval is "<<fan_dcval[1]<<"!!!"<<std::endl;
        fan_dcval_switch = true;
    }
    else if(1 == msg.buttons[BUTTON_SHAPE_B]){

        fan_3_dcval_switch = ! fan_3_dcval_switch;
        fan_dcval[2] = (240.0 * fan_3_dcval_switch);
        std::cout<<"The current fan_3_dcval is "<<fan_dcval[2]<<"!!!"<<std::endl;
        fan_dcval_switch = true;
    }
    else if(1 == msg.buttons[BUTTON_RT]){
        for(int i = 0; i < 3; i++){
                fan_dcval[i] = 0.0;    //全部风机关闭
        }
        std::cout<<"The current fan_dcval has been reset to 0 !!!"<<std::endl;
        fan_dcval_switch = true; 
    }

    if(1 == msg.buttons[BUTTON_SHAPE_START]){
        joy_switch_state = !joy_switch_state;
        std::cout<<"current joy switch state is : "<<joy_switch_state<<"!!!"<<&std::endl;
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "logicool_controller_apb");
    ros::NodeHandle n;
    ros::Rate loopRate(50);

    ros::Subscriber joy_sub = n.subscribe("/joy",1000,JoyCallback);

    ros::Publisher MotorSpeed_pub = n.advertise<apb_msgs::MotorSpeed>("/command/Motor_Speed", 1);  
    ros::Publisher MotorPos_pub = n.advertise<apb_msgs::MotorPos>("/command/Motor_Position", 1);  
    ros::Publisher MotorDC_pub = n.advertise<apb_msgs::MotorDCval>("/command/Motor_Dcval", 1);   
    ros::Publisher FanVel_pub = n.advertise<apb_msgs::FanDC>("/command/Fan_velocity", 1); 

    apb_msgs::MotorSpeed motor_speed_msg;
    apb_msgs::MotorPos motor_pos_msg;
    apb_msgs::MotorDCval motor_dcval_msg;
    apb_msgs::FanDC fan_velocity_msg;



    while(ros::ok()){

        //轮速
        if(joy_switch_state && motor_speed_switch){
            motor_speed_msg.motor1_speed = motor_speed[0];
            motor_speed_msg.motor2_speed = motor_speed[1];
            motor_speed_msg.motor3_speed = motor_speed[2];
            MotorSpeed_pub.publish(motor_speed_msg);
            motor_speed_switch = false;
        }

        //转向
        if(joy_switch_state && motor_pos_switch){
            motor_pos_msg.motor1_pos = motor_pos[0];
            motor_pos_msg.motor2_pos = motor_pos[1];
            motor_pos_msg.motor3_pos = motor_pos[2];
            MotorPos_pub.publish(motor_pos_msg);
            motor_pos_switch = false;
        }

        //水下吸附
        if(joy_switch_state && motor_dcval_switch){
            motor_dcval_msg.motor1_dcval = motor_dcval[0];
            motor_dcval_msg.motor2_dcval = motor_dcval[1];
            motor_dcval_msg.motor3_dcval = motor_dcval[2];
            MotorDC_pub.publish(motor_dcval_msg);
            motor_dcval_switch = false;
        }

        //水上吸附
        if(joy_switch_state && fan_dcval_switch){
            fan_velocity_msg.fan1_dcval = fan_dcval[0];
            fan_velocity_msg.fan2_dcval = fan_dcval[1];
            fan_velocity_msg.fan3_dcval = fan_dcval[2];
            FanVel_pub.publish(fan_velocity_msg);
            fan_dcval_switch = false;
        }

        // if(light_intensity_switch){
        //     light_intensity_msg.data = light_intensity;
        //     light_intensity_pub.publish(light_intensity_msg);
        //     light_intensity_switch = false;
        // }

        // if(mad_velocity_switch){
        //     mad_velocity_msg.data = mad_velocity;
        //     mad_velocity_pub.publish(mad_velocity_msg);
        //     mad_velocity_switch = false;
        // }

        // output_netLoad.header.stamp = ros::Time::now();
        // //Move forward
        // output_netLoad.force.x = joy_force[0];
        // output_netLoad.force.y = joy_force[1];
        // output_netLoad.force.z = joy_force[2];
        // //Rotate around z axis (yaw)
        // output_netLoad.moment.x = joy_moment[0];
        // output_netLoad.moment.y = joy_moment[1];
        // output_netLoad.moment.z = joy_moment[2];
        // joy_netLoad_pub.publish(output_netLoad);
        ros::spinOnce();
        loopRate.sleep();
    }
}


