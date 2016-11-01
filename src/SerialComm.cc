/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file SerialComm.cc
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "MAVLinkBridge.h"
#include "SerialComm.h"

#include <fcntl.h>

#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

//-----------------------------------------------------------------------------
SerialComm::SerialComm(bool raw)
    : CommLink(raw)
    , _fd(-1)
    , _baudrate(57600)
    , _status(SERIAL_PORT_CLOSED)
{

}

//-----------------------------------------------------------------------------
SerialComm::~SerialComm()
{
    close();
}

//-----------------------------------------------------------------------------
bool
SerialComm::init(std::string port)
{
    _uart_name = port;
    int idx = port.find(':');
    if(idx >= 0) {
        _uart_name = port.substr(0, idx);
        _baudrate  = atoi(port.substr(idx+1).c_str());
        if(!_baudrate) {
            _baudrate = 57600;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool
SerialComm::open()
{
    if(_status != SERIAL_PORT_CLOSED || _fd >= 0) {
        return false;
    }
    _fd = _openPort(_uart_name.c_str());
    if(_fd < 0) {
        perror("SERIAL");
        fprintf(stderr, "SERIAL: Could not open port %s.\n", _uart_name.c_str());
        return false;
    }
    tcgetattr(_fd , &_savedtio);
    if(!_setupPort(_baudrate)) {
        return false;
    }
    _status = SERIAL_PORT_OPEN;
    return true;
}

//-----------------------------------------------------------------------------
void
SerialComm::close()
{
    if(_fd >= 0) {
#ifdef __APPLE__
        tcflush(_fd, TCIFLUSH);
#endif
        tcsetattr(_fd, TCSANOW, &_savedtio);
        ::close(_fd);
        _fd = -1;
    }
    _status = SERIAL_PORT_CLOSED;
    join();
}

//-----------------------------------------------------------------------------
void
SerialComm::write(mavlink_message_t &message)
{
    char buf[1024];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    _writePort(buf, len);
}

//-----------------------------------------------------------------------------
void
SerialComm::write(std::vector<uint8_t> data)
{
    _writePort(data.data(), data.size());
}

//-----------------------------------------------------------------------------
void
SerialComm::run()
{
    if(_rawMode) {
        while(_status == SERIAL_PORT_OPEN) {
            uint8_t buffer[128];
            int count = ::read(_fd, buffer, sizeof(buffer));
            if(count > 0) {
                std::vector<uint8_t>* data = new std::vector<uint8_t>(count);
                memcpy(data->data(), buffer, count);
                _mutex.lock();
                _inMessagesRaw.push(data);
                _mutex.unlock();
                pApp->event.set();
            }
        }
    } else {
        mavlink_message_t* message = NULL;
        while(_status == SERIAL_PORT_OPEN) {
            if(!message) {
                message = (mavlink_message_t*)malloc(sizeof(mavlink_message_t));
            }
            if(_readMessage(*message)) {
                _mutex.lock();
                //-- If we start accumulating too many messages, get rid of them
                if(_inMessages.size() > 1000) {
                    mavlink_message_t* discard = _inMessages.front();
                    free(discard);
                    _inMessages.pop();
                }
                _inMessages.push(message);
                message = NULL;
                _mutex.unlock();
                pApp->event.set();
            } else {
                if(_status != SERIAL_PORT_OPEN) {
                    free(message);
                    break;
                }
            }
        }
    }
    fprintf(stderr, "SERIAL: Exiting thread\n");
}

//-----------------------------------------------------------------------------
int
SerialComm::_readMessage(mavlink_message_t& message)
{
    uint8_t msgReceived = 0;
    while(!msgReceived) {
        uint8_t cp;
        int result = ::read(_fd, &cp, 1);
        if(result > 0) {
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &_mstatus);
        } else {
            break;
        }
    }
    return msgReceived;
}

//-----------------------------------------------------------------------------
int
SerialComm::_openPort(const char* port)
{
    int fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd >= 0) {
        fcntl(fd, F_SETFL, 0);
    }
    return fd;
}

//-----------------------------------------------------------------------------
bool
SerialComm::_setupPort(int baud)
{
    struct termios config;
    bzero(&config, sizeof(config));
    config.c_cflag |= (CS8 | CLOCAL | CREAD);
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 5;
    bool baudError = false;
    switch(baud) {
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            baudError = (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0);
            break;
        case 57600:
            baudError = (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0);
            break;
        case 115200:
            baudError = (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0);
            break;
        case 460800:
            baudError = (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0);
            break;
        case 921600:
            baudError = (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0);
            break;
        default:
            baudError = false;
            break;
    }
    if(baudError) {
        fprintf(stderr, "SERIAL: Could not set baud rate of %d\n", baud);
        return false;
    }
    tcflush(_fd, TCIFLUSH);
    if(tcsetattr(_fd, TCSANOW, &config) < 0) {
        fprintf(stderr, "SERIAL: Could not set serial configuration\n");
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
int
SerialComm::_writePort(void* buffer, int len)
{
    int written = ::write(_fd, buffer, len);
    if(written != len && written >= 0) {
        fprintf(stderr, "SERIAL: Wrote only %d bytes out of %d\n", written, len);
    }
    return written;
}
