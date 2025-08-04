#include "ros/ros.h"
#include "string"
#include "stdio.h"
#include "serial/serial.h"
#include "apb_msgs/FanDC.h"
#include "apb_msgs/MotorPos.h"
#include "apb_msgs/MotorDCval.h"
#include "apb_msgs/MotorSpeed.h"
#include "std_msgs/Int64.h"

#define mspeed_buffer_size 14
unsigned char mspeed_buffer[mspeed_buffer_size];
#define mpos_buffer_size 14
unsigned char mpos_buffer[mpos_buffer_size];
#define motorDC_buffer_size 8
unsigned char motorDC_buffer[motorDC_buffer_size];
#define fan_buffer_size 8
unsigned char fan_buffer[fan_buffer_size];

using namespace std;

serial::Serial ser;

int data_u8[4] = {0,0,0,0};
int pwm_data_high = 0;
int pwm_data_low = 0;

//32 to 8
void dcvalToHex(int data_input){
    data_u8[0] = (data_input >> 24) & 0xff;	 
    data_u8[1] = (data_input >> 16) & 0xff; 	   
    data_u8[2] = (data_input >> 8)  & 0xff;
    data_u8[3] =  data_input        & 0xff;
}

// Dec to Hex  16 to 8
void pwmToHex(int pwm_input){
    pwm_data_high = pwm_input / (16 * 16);
    pwm_data_low = pwm_input % (16 * 16);
}


//轮速 4Byte -50~50
void sendMotorSpeed(const apb_msgs::MotorSpeed& msg){
    memset(mspeed_buffer, 0, sizeof(mspeed_buffer));

    mspeed_buffer[0] = 0x01;
    mspeed_buffer[1] = 0x02;
    
    dcvalToHex(msg.motor1_speed);
    mspeed_buffer[2] = 0x00 + data_u8[0];
    mspeed_buffer[3] = 0x00 + data_u8[1];
    mspeed_buffer[4] = 0x00 + data_u8[2];
    mspeed_buffer[5] = 0x00 + data_u8[3];

    dcvalToHex(msg.motor2_speed);
    mspeed_buffer[6] = 0x00 + data_u8[0];
    mspeed_buffer[7] = 0x00 + data_u8[1];
    mspeed_buffer[8] = 0x00 + data_u8[2];
    mspeed_buffer[9] = 0x00 + data_u8[3];

    dcvalToHex(msg.motor3_speed);
    mspeed_buffer[10] = 0x00 + data_u8[0];
    mspeed_buffer[11] = 0x00 + data_u8[1];
    mspeed_buffer[12] = 0x00 + data_u8[2];
    mspeed_buffer[13] = 0x00 + data_u8[3];

    ser.write(mspeed_buffer, mspeed_buffer_size);

    ROS_INFO("setMotorSpeed: motor1-> %dmm/s motor2->%dmm/s motor3->%dmm/s", \
    msg.motor1_speed, msg.motor2_speed, msg.motor3_speed);
}

//转向 4Byte -90~90
void sendMotorPos(const apb_msgs::MotorPos& msg){
    memset(mpos_buffer, 0, sizeof(mpos_buffer));

    mpos_buffer[0] = 0x02;
    mpos_buffer[1] = 0x03;
    
    dcvalToHex(msg.motor1_pos);
    mpos_buffer[2] = 0x00 + data_u8[0];
    mpos_buffer[3] = 0x00 + data_u8[1];
    mpos_buffer[4] = 0x00 + data_u8[2];
    mpos_buffer[5] = 0x00 + data_u8[3];

    dcvalToHex(msg.motor2_pos);
    mpos_buffer[6] = 0x00 + data_u8[0];
    mpos_buffer[7] = 0x00 + data_u8[1];
    mpos_buffer[8] = 0x00 + data_u8[2];
    mpos_buffer[9] = 0x00 + data_u8[3];

    dcvalToHex(msg.motor3_pos);
    mpos_buffer[10] = 0x00 + data_u8[0];
    mpos_buffer[11] = 0x00 + data_u8[1];
    mpos_buffer[12] = 0x00 + data_u8[2];
    mpos_buffer[13] = 0x00 + data_u8[3];

    ser.write(mpos_buffer, mpos_buffer_size);
    if(msg.motor1_pos == 1.0){
         ROS_INFO("机器人正在右转！");
    }
    else if(msg.motor1_pos == 2.0){
         ROS_INFO("机器人正在左转！");
    }
    else{
        ROS_INFO("机器人正在回正！");
    }

    // ROS_INFO("setMotorPos: motor1-> %d motor2->%d motor3->%d", \
    // msg.motor1_pos, msg.motor2_pos, msg.motor3_pos);

}

//水下电机 2Byte 1250-1750
void sendMotorDCval(const apb_msgs::MotorDCval& msg){
    memset(motorDC_buffer, 0, sizeof(motorDC_buffer));

    motorDC_buffer[0] = 0x03;
    motorDC_buffer[1] = 0x04;
    
    pwmToHex(msg.motor1_dcval);
    motorDC_buffer[2] = 0x00 + pwm_data_high;
    motorDC_buffer[3] = 0x00 + pwm_data_low;

    pwmToHex(msg.motor2_dcval);
    motorDC_buffer[4] = 0x00 + pwm_data_high;
    motorDC_buffer[5] = 0x00 + pwm_data_low;

    pwmToHex(msg.motor3_dcval);
    motorDC_buffer[6] = 0x00 + pwm_data_high;
    motorDC_buffer[7] = 0x00 + pwm_data_low;

    ser.write(motorDC_buffer, motorDC_buffer_size);

    ROS_INFO("setMotorDCval: motor1-> %d motor2->%d motor3->%d", \
    msg.motor1_dcval, msg.motor2_dcval, msg.motor3_dcval);
}

//风机 2Byte 0-1000
void sendFanDCval(const apb_msgs::FanDC& msg){
    memset(fan_buffer, 0, sizeof(fan_buffer));

    fan_buffer[0] = 0x04;
    fan_buffer[1] = 0x05;

    pwmToHex(msg.fan1_dcval);
    fan_buffer[2] = 0x00 + pwm_data_high;
    fan_buffer[3] = 0x00 + pwm_data_low;

    pwmToHex(msg.fan2_dcval);
    fan_buffer[4] = 0x00 + pwm_data_high;
    fan_buffer[5] = 0x00 + pwm_data_low;

    pwmToHex(msg.fan3_dcval);
    fan_buffer[6] = 0x00 + pwm_data_high;
    fan_buffer[7] = 0x00 + pwm_data_low;  
    
    ser.write(fan_buffer, fan_buffer_size);

    ROS_INFO("setFanDCval: motor1-> %d motor2->%d motor3->%d", \
    msg.fan1_dcval, msg.fan2_dcval, msg.fan3_dcval);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "serial_to_mcu_apb");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string serial_port_;
    int baudrate_;
    nh_private.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    nh_private.param<int>("baudrate", baudrate_, 115200);
    
    ros::Subscriber MotorSpeed_sub = nh.subscribe("/command/Motor_Speed", 1000, sendMotorSpeed);   
    ros::Subscriber MotorPos_sub = nh.subscribe("/command/Motor_Position", 1000, sendMotorPos);  
    ros::Subscriber MotorDC_sub = nh.subscribe("/command/Motor_Dcval", 1000, sendMotorDCval);  
    ros::Subscriber FanVel_sub = nh.subscribe("/command/Fan_velocity", 1000, sendFanDCval); 

    try{
        ser.setPort(serial_port_);
        ser.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port!!!");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized...");
    }
    else{
        return -1;
    }

    ros::Rate loop_rate(50);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

