//
// Created by zhaomumu on 19-5-6.
//

#include <ros/ros.h>
#include <remote_car/car_data.h>
#include <string>
#include <iostream>
#include <cstdio>

#include <unistd.h>
#include "serial.h"

#include <opencv2/opencv.hpp>

using namespace std;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


// union 中的数据共享同一段内存,union变量所占用的内存长度等于最长的成员的内存长度。
union channel{
    //union 在存储结构上,高八位 ch_8_t[1] 在高地址, 低八位 ch_8_t[0] 在低地址
    uint16_t _16_t;
    uint8_t _8_t[2];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote_msg_publisher");
    ros::NodeHandle nh;
    ros::Publisher remote_msg_pub = nh.advertise<remote_car::car_data>("car_remote_data",1000);

    unsigned long baud = 115200;
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    string trueport;
    int trueflag = false;
    int while_count = 0 ;

    while( iter != devices_found.end() && (trueflag== false))
    {
        serial::PortInfo device = *iter++;
        printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
                device.hardware_id.c_str() );
        string port_name = device.port;
        serial::Serial test_serial(port_name,baud);
        if(!test_serial.isOpen())
            continue;
        else //打开串口
        {
            std::cout<<"open port "<<port_name<<std::endl;
            int rx_flag = -1 ;
            uint8_t rx_buf[35];

            size_t size = 70;
            uint8_t *buffer = new uint8_t[size];
            size_t result = test_serial.read(buffer,size);

            while((result<40)&&(while_count<10)){
                result = test_serial.read(buffer,size);
                while_count++;
                usleep(1000);
            }

            std::cout<<"get buffer num : "<<result<<std::endl;
            if((result!=0) && (trueflag==false)){
                std::cout<<std::hex;
                for(int i=0;i<result;i++)
                {
                    if (rx_flag<100)
                    {
                        if (rx_flag>9)
                        {
                            rx_flag++;
                            uint8_t temp = rx_flag-10;
                            rx_buf[temp] = *buffer;

                            if (temp == 34) {
                                uint8_t xor_v = 0;
                                for (int i = 1; i < 34; i++) {
                                    xor_v = xor_v ^ rx_buf[i];
                                }
                                if (*buffer == xor_v) {
                                    rx_flag = 255;
                                    for(int i = 0 ; i < 35 ; i++){
                                        std::cout<<std::hex;
                                    }

                                } else{
                                    rx_flag = -1;
                                }
                            }
                        } else { //新的一帧数据开始接收
                            if (*buffer == 0x0f)//校验帧头
                            {
                                rx_buf[0] = *buffer;
                                rx_flag = 10;
                            } else {
                                rx_flag = 0 ;
                            }
                        }
                    }
                    if (rx_flag==255)
                    {
                        trueport = port_name;
                        trueflag = true;
                        break;
                    }
                    buffer++;
                }
            }
        }
    }

    if(trueflag==false)
    {
        std::cout<<"cannot get the true port . exit 0 "<<std::endl;
        return 0 ;
    }
    std::cout<<"End Check . The True Port is "<< trueport <<std::endl;

    string port(trueport);

    serial::Serial my_serial(port, baud);

    int rx_flag = -1 ;
    uint8_t rx_buf[35];
    uint16_t channel_value[16];
    bool loop = false;

    ros::Rate loop_rate(30);

    while(ros::ok()){

        while(loop==false)
        {
            size_t size = 1;
            uint8_t *buffer = new uint8_t[size];

            if(!my_serial.isOpen())
                my_serial.open();

            size_t result = my_serial.read(buffer,size);
            if (result != 0) //读取到数据
            {
                if (rx_flag<100)
                {
                    if (rx_flag>9)
                    {
                        rx_flag++;
                        uint8_t temp = rx_flag-10;
                        rx_buf[temp] = *buffer;

                        if (temp == 34) {
                            uint8_t xor_v = 0;
                            for (int i = 1; i < 34; i++) {
                                xor_v = xor_v ^ rx_buf[i];
                            }
                            if (*buffer == xor_v) {
                                rx_flag = 255;
                                for(int i = 0 ; i < 35 ; i++){
                                    std::cout<<std::hex;
                                }

                            } else{
                                rx_flag = -1;
                            }
                        }
                    } else { //新的一帧数据开始接收
                        if (*buffer == 0x0f)//校验帧头
                        {
                            rx_buf[0] = *buffer;
                            rx_flag = 10;
                        } else {
                            rx_flag = 0 ;
                        }
                    }
                }
                if (rx_flag==255) //接收数据完成,进行通道数据转换
                {
                    my_serial.close();

                    remote_car::car_data msg;

                    cv::Mat image = cv::Mat(500,500,CV_8UC3,cv::Scalar(255,255,255));
                    std::string str[16];
                    for (int i = 1; i < 32; i = i + 2) {
                        int index = (i - 1) / 2;
                        channel_value[index] = (rx_buf[i] << 8) | rx_buf[i + 1];
                    }
                    /*!
                     * @param: ch1 368--1680 对应方向盘的 780 --（-780）
                     * */
                    if(channel_value[0]<=1012 && channel_value[0]>=1008)
                        msg.steering_wheel = 0;
                    else if(channel_value[0]>1012){

                        float temp = -(((float)channel_value[0]-1012.)/668.) * 780.;
                        msg.steering_wheel = (int16_t)temp ;
                    }
                    else if(channel_value[0]<1008){
                        float temp = ((1008.-(float)channel_value[0])/640.) * 780.;
                        msg.steering_wheel = (int16_t)temp ;
                    }
                    else
                        msg.steering_wheel = 0;

                    /*********************************************/
                    // 增加单个开关以请求进入转向自动控制模式
                    // 开关D 向上拨为0  向下拨为1
                    if(channel_value[9]<500)
                        msg.Enter_steering_control = 0;
                    else if(channel_value[9]>1500)
                        msg.Enter_steering_control = 1;
                    else
                        msg.Enter_steering_control = 0;

                    //增加左手 油门 刹车 急刹 档位
                    /*!
                     * @note: 刹车 往前374 中间
                     * @note: 急刹 开关F 往上拨 1； 往下拨 0
                     * */
                    msg.accelerator = 0 ; //油门 int16_t 0-2000
                    msg.emergency_braking = 0 ; //急刹 开关F 向上拨急刹1 向下拨:正常状态0
                    msg.brake = 0; //刹车 float 32位 0-99.0
                    msg.gear = 0 ; //档位 开关C 中间-0空挡 向前拨-1前进档 向后拨-2后退档

                    int16_t brake_accelerator_ = (int16_t)channel_value[1];
                    int16_t emergency_braking_ = (int16_t)channel_value[4];
                    int16_t gear_ = (int16_t)channel_value[6];

                    if(gear_<200)
                    {
                        if(brake_accelerator_>=1050 && brake_accelerator_<=1688){
                            msg.accelerator = (brake_accelerator_-1050)*3;
                        }
                        else
                            msg.accelerator = 0 ;

                        msg.gear = 1; //前进档
                    }
                    else if(gear_>1800)
                    {
                        if(brake_accelerator_>=1050 && brake_accelerator_<=1688){
                            msg.accelerator = -(brake_accelerator_-1050)*3;
                        }
                        else
                            msg.accelerator = 0 ;

                        msg.gear = 2; //后退档
                    }
                    else{
                        msg.gear = 0; //空挡
                    }


                    if(emergency_braking_<200)
                    {
                        //正常控制状态
                        if(brake_accelerator_<=1020 && brake_accelerator_>=374){
                            msg.brake = float(1020.-brake_accelerator_)*99./646. ;
                            msg.accelerator = 0;
                        }
                        msg.emergency_braking = 0 ; //非急刹
                    }
                    else if (emergency_braking_>1800)
                    {
                        //急刹
                        msg.accelerator = 0; //油门给0
                        msg.brake = 99.0 ;  //刹车给满
                        msg.emergency_braking = 1 ; //急刹
                    }
                    else ;

                    /*********************************************/

                    string str1;
                    str1 = "steering_wheel: " + std::to_string(msg.steering_wheel);
                    cv::putText(image,str1,cv::Point2d(50,50),1,1,cv::Scalar(0,0,0));

                    /*********************************************/
                    // 增加显示
                    string str2;
                    str2 = "Enter_steering_control: "+std::to_string(msg.Enter_steering_control);
                    cv::putText(image,str2,cv::Point2d(50,100),1,1,cv::Scalar(0,0,0));
                    string str3;
                    str3 = "accelerator: "+std::to_string(msg.accelerator);
                    cv::putText(image,str3,cv::Point2d(50,150),1,1,cv::Scalar(0,0,0));
                    string str4;
                    str4 = "emergency_braking: "+std::to_string(msg.emergency_braking);
                    cv::putText(image,str4,cv::Point2d(50,200),1,1,cv::Scalar(0,0,0));

                    string str5;
                    str4 = "brake: "+std::to_string(msg.brake);
                    cv::putText(image,str4,cv::Point2d(50,250),1,1,cv::Scalar(0,0,0));

                    string str6;
                    str4 = "gear: "+std::to_string(msg.gear);
                    cv::putText(image,str4,cv::Point2d(50,300),1,1,cv::Scalar(0,0,0));
                    /*********************************************/

                    cv::imshow("image",image);
                    cv::waitKey(1);
                    rx_flag = -1;

                    remote_msg_pub.publish(msg);

                    break;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
}