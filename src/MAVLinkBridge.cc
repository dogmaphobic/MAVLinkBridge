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
 * @file MAVLinkBridge.cc
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "MAVLinkBridge.h"
#include "SerialComm.h"
#include "UDPComm.h"

#include <algorithm>

//-----------------------------------------------------------------------------
class CmdLineParser{
    public:
        CmdLineParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->_vArgs.push_back(std::string(argv[i]));
        }
        const std::string& getOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->_vArgs.begin(), this->_vArgs.end(), option);
            if (itr != this->_vArgs.end() && ++itr != this->_vArgs.end()){
                return *itr;
            }
            return _nothing;
        }
        bool exists(const std::string &option) const{
            return std::find(this->_vArgs.begin(), this->_vArgs.end(), option)
                   != this->_vArgs.end();
        }
    private:
        std::vector <std::string> _vArgs;
        std::string _nothing;
};

//-----------------------------------------------------------------------------
MAVLinkBridge::MAVLinkBridge()
    : _serialComm(NULL)
    , _udpComm(NULL)
    , _running(false)
    , _rawMode(false)
{
}

//-----------------------------------------------------------------------------
MAVLinkBridge::~MAVLinkBridge()
{
    if(_serialComm) {
        delete _serialComm;
    }
    if(_udpComm) {
        delete _udpComm;
    }
}

//-----------------------------------------------------------------------------
int MAVLinkBridge::run(int argc, char *argv[])
{
    bool switchToMulticast = false;
    CmdLineParser input(argc, argv);
    if(input.exists("-h")) {
        _usage(argv[0]);
        return 0;
    }
    if(input.exists("-r")) {
        _rawMode = true;
    }
    if(input.exists("-w")) {
        switchToMulticast = true;
    }
    const std::string &udpString = input.getOption("-u");
    if (udpString.empty()){
        fprintf(stderr, "Missing UDP argument\n");
        _usage(argv[0]);
        return 0;
    }
    const std::string &serialString = input.getOption("-s");
    if (serialString.empty()){
        fprintf(stderr, "Missing Serial argument\n");
        _usage(argv[0]);
        return 0;
    }
    _serialComm = new SerialComm(_rawMode);
    if(_serialComm->init(serialString) && _serialComm->open()) {
        _udpComm = new UDPComm(_rawMode, switchToMulticast);
        if(_udpComm->init(udpString) && _udpComm->open()) {
            _running = true;
            _serialComm->startThread();
            _udpComm->startThread();
            if(_rawMode)
                return _loopRaw();
            else
                return _loop();
        }
    }
    return -1;
}

//-----------------------------------------------------------------------------
void
MAVLinkBridge::_usage(const char* app)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "%s [options] -s serial_port[:baud] -u target_address[:port]\n", app);
    fprintf(stderr, "\nOptions:\n");
    fprintf(stderr, "-r          Raw mode (don't break into mavlink message chunks)\n");
    fprintf(stderr, "-s name     Serial port name (and optionally the baud rate)\n");
    fprintf(stderr, "-u address  IP address or host name (and optionally the port number)\n");
    fprintf(stderr, "-w          Switch to unicast mode once a response is received\n");
    fprintf(stderr, "\nExample:\n");
    fprintf(stderr, "\n  %s -s /dev/tty.usbmodem1:115200 -u 192.168.1.255 -r -w\n", app);
    fprintf(stderr, "\n  Starts a bridge between a vehicle on /dev/tty.usbmodem1 at 115200 baud and a\n"\
                      "  GCS on the 192.168.1 network (broadcast). Use raw mode and switch to unicast\n"\
                      "  once a response from the GCS is received.\n\n");
}

//-----------------------------------------------------------------------------
int
MAVLinkBridge::_loop()
{
    while(_running) {
        bool busy = false;
        mavlink_message_t* message = _serialComm->read();
        if(message) {
            _udpComm->write(*message);
            delete message;
            busy = true;
        }
        message = _udpComm->read();
        if(message) {
            _serialComm->write(*message);
            delete message;
            busy = true;
        }
        if(!busy) {
            event.wait(100);
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
int
MAVLinkBridge::_loopRaw()
{
    while(_running) {
        bool busy = false;
        std::vector<uint8_t>* data = _serialComm->readRaw();
        if(data) {
            _udpComm->write(*data);
            delete data;
            busy = true;
        }
        data = _udpComm->readRaw();
        if(data) {
            _serialComm->write(*data);
            delete data;
            busy = true;
        }
        if(!busy) {
            event.wait(100);
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
void
MAVLinkBridge::stop()
{
    _serialComm->close();
    _udpComm->close();
    _running = false;
    event.set();
}
