//
// Created by zhaomumu on 19-5-6.
//

#include "open_serial.h"

//
// Created by zhaomumu on 19-4-26.
//

#include <stdio.h>
#include <stropts.h>

/**
 * @file Serial.cpp
 * @brief Simple serial interface, for example to talk to Arduino.
 * @author: Michael Kaess
 */

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include "open_serial.h"

using namespace std;


// open a serial port connection
void OpenSerial::open(const string& port, int rate)
{
    m_serialPort = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_serialPort==-1)
    {
        cout << "Unable to open serial port" << endl;
        exit(1);
    }
    fcntl(m_serialPort, F_SETFL,0); // O_NONBLOCK might be needed for write...

    struct termios port_settings;      // structure to store the port settings in
    tcgetattr(m_serialPort, &port_settings); // get current settings

    speed_t b;
    switch(rate)
    {
        case(9600):
            b = B9600;
            break;
        case(19200):
            b = B19200;
            break;
        case(38400):
            b = B38400;
            break;
        case(115200):
            b = B115200;
            break;
        case(1000000):
            b = B1000000;
            break;
        default:
            cout << "Error: Unknown baud rate requested in Serial.open()" << endl;
            exit(1);
    }

    cfsetispeed(&port_settings, b);    // set baud rates
    cfsetospeed(&port_settings, b);

    port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, 8 data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    tcsetattr(m_serialPort, TCSANOW, &port_settings);    // apply the settings to the port
}

// read a single character
uint8_t* OpenSerial::read() const
{
    uint8_t* result;
    if (::read(m_serialPort, result, 1) == 1) {
        return result;
    } else {
        return nullptr;
    }
}

// read until special character up to a maximum number of bytes
//string OpenSerial::readBytesUntil(unsigned char until, int max_length)
//{
//    string result(max_length, ' ');
//    int n = 0;
//    int c;
//    do {
//        c = read();
//        if (c<0) { // wait for more characters
//            usleep(100);
//        } else {
//            result[n] = (unsigned char)c;
//            n++;
//        }
//    } while ((c != (int)until) && (n < max_length));
//    result.resize(n);
//    return result;
//}

// send a string
void OpenSerial::print(string str) const
{
    int res = ::write(m_serialPort, str.c_str(), str.length());
}

// send an integer
void OpenSerial::print(int num) const {
    stringstream stream;
    stream << num;
    string str = stream.str();
    print(str);
}

// send a double
void OpenSerial::print(double num) const {
    stringstream stream;
    stream << num;
    string str = stream.str();
    print(str);
}

// send a float
void OpenSerial::print(float num) const {
    print((double)num);
}

