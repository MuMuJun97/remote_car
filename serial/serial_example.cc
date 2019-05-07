#include <string>
#include <iostream>
#include <cstdio>

#include <unistd.h>
#include "serial.h"

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




#include <opencv2/opencv.hpp>
#include <chrono>
using namespace std::chrono;
int main(int argc, char **argv) {
    unsigned long baud = 115200;
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    string trueport;
    int trueflag = false;
    int while_count = 0 ;

    auto start = chrono::system_clock::now();

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

            size_t size = 64;
            uint8_t *buffer = new uint8_t[size];
            size_t result = test_serial.read(buffer,size);

            while((result<40)&&(while_count<10)){
                result = test_serial.read(buffer,size);
                while_count++;
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

    auto end = chrono::system_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end- start);
    cout<<"Check cost :"<<double(duration.count())*chrono::microseconds::period::num/chrono::microseconds::period::den<<endl;

    if(trueflag==false)
        return 0 ;

    std::cout<<"End Check . The True Port is "<< trueport <<std::endl;

    string port(trueport);

    serial::Serial my_serial(port, baud);

    int rx_flag = -1 ;
    uint8_t rx_buf[35];
    uint16_t channel_value[16];

    int time_num = 0 ;

    while(1){

        size_t size = 1;
        uint8_t *buffer = new uint8_t[size];
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
                        start = chrono::system_clock::now();
                    } else {
                        rx_flag = 0 ;
                    }
                }
            }
            if (rx_flag==255) //接收数据完成,进行通道数据转换
            {
                std::string str[16];
                for (int i = 1; i < 32; i = i + 2) {
                    int index = (i - 1) / 2;
                    channel_value[index] = (rx_buf[i] << 8) | rx_buf[i + 1];
                }
                std::cout<<"\n 16 channels value: \n";
                std::cout<<std::dec;
                for(int i = 0 ; i < 16 ; i++){

                    if(i==3){
                        std::cout<<"Right__Up-Down: "<<channel_value[i-1]<<" ; \n";
                    }
                    if(i==1){
                        std::cout<<"Right__right-left: "<<channel_value[i-1]<<" ; \n ";
                    }
                    if(i==2){
                        std::cout<<"Left__Up-Down: "<<channel_value[i-1]<<" ; \n";
                    }
                    if(i==4){
                        std::cout<<"Left__right-left: "<<channel_value[i-1]<<" ; \n";
                    }

                }
                rx_flag = -1;
                std::cout<<"\n\n";
                end = chrono::system_clock::now();
                cout<<"Check cost :"<<double(duration.count())*chrono::microseconds::period::num/chrono::microseconds::period::den<<endl;
                time_num++;
                cout<<"Time num "<<time_num<<endl;
            }
        }


    }
}
