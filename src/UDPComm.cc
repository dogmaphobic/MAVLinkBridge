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
 * @file UDPComm.cc
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "MAVLinkBridge.h"
#include "UDPComm.h"

//-----------------------------------------------------------------------------
UDPComm::UDPComm(bool raw, bool switchToUnicast)
    : CommLink(raw)
    , _socket(-1)
    , _valid(false)
    , _switchToUnicast(switchToUnicast)
    , _inBufferIndex(0)
    , _inBufferCount(0)
{
}

//-----------------------------------------------------------------------------
UDPComm::~UDPComm()
{
    close();
}

//-----------------------------------------------------------------------------
void
UDPComm::write(mavlink_message_t& message)
{
    char buf[1024];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    if(!_sendTo(buf, len)) {
        fprintf(stderr, "UDP: Error writing to socket.\n");
    }
}

//-----------------------------------------------------------------------------
void
UDPComm::write(std::vector<uint8_t> data)
{
    if(!_sendTo(data.data(), data.size())) {
        fprintf(stderr, "UDP: Error writing to socket.\n");
    }
}

//-----------------------------------------------------------------------------
bool
UDPComm::init(std::string address)
{
    uint16_t port = MVB_UDP_TARGET_PORT;
    std::string caddress = address;
    int idx = address.find(':');
    if(idx > 0) {
        caddress = address.substr(0, idx);
        port = atoi(address.substr(idx + 1).c_str());
    }
    if(!_setTargetAddress(caddress, port)) {
        fprintf(stderr, "UDP: Address not found: %s\n", address.c_str());
        return false;
    }
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if(_socket < 0) {
        fprintf(stderr, "UDP: Error creating socket.\n");
        _valid = false;
    } else {
        _valid = true;
        int i = 1; //-- Allow broadcasting
        setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, SOCKOPTVAL & i, sizeof(i));
#if defined(__APPLE__)
        int s = 1;  //-- Do Not generate SIGPIPE on closed socket
        setsockopt(_socket, SOL_SOCKET, SO_NOSIGPIPE, SOCKOPTVAL & s, sizeof(s));
#endif
        unsigned long b = 1; // Don't Block
        ioctl(_socket, FIONBIO, &b);
    }
    return _valid;
}

//-----------------------------------------------------------------------------
bool
UDPComm::open()
{
    if(_valid) {
        _valid = _bind();
    }
    return _valid;
}

//-----------------------------------------------------------------------------
void
UDPComm::close()
{
    if(_socket >= 0) {
        shutdown(_socket, 1);
        ::close(_socket);
        _socket = -1;
    }
    _valid = false;
    join();
}

//-----------------------------------------------------------------------------
void
UDPComm::run()
{
    if(_rawMode) {
        while(_valid) {
            int count = _read(_inBuffer, sizeof(_inBuffer));
            if(count > 0) {
                std::vector<uint8_t>* data = new std::vector<uint8_t>(count);
                memcpy(data->data(), _inBuffer, count);
                _mutex.lock();
                _inMessagesRaw.push(data);
                _mutex.unlock();
                pApp->event.set();
            }
        }
    } else {
        mavlink_message_t* message = NULL;
        while(_valid) {
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
                if(!_valid) {
                    free(message);
                    break;
                }
            }
        }
    }
    fprintf(stderr, "UDP: Exiting thread\n");
}

//-----------------------------------------------------------------------------
bool
UDPComm::_sendTo(void* buffer, int len)
{
    char* ptr = (char*)buffer;
    int count = 0;
    int total = 0;
    int left = len;
    do {
        int written = ::sendto(_socket, (IOTYPE)ptr, left, 0, (struct sockaddr*)&_targetAddress, sizeof(_targetAddress));
        if(written < 0) {
            int error = GIOERRNO;
            if (error == GEAGAIN) {
                if (!_writeSelect()) {
                    return false;
                }
                continue;
            } else {
                return false;
            }
        } else {
            total += count;
            if (count && count < int(left)) {
                left -= count;
                ptr  += count;
            } else
                break;
        }
    } while(true);
    return true;
}

//-----------------------------------------------------------------------------
bool
UDPComm::_setTargetAddress(const std::string& host, uint16_t port)
{
    if(host.size() > 255 || !host.size()) {
        return false;
    }
    struct hostent* hp;
    struct in_addr** bptr;
    in_addr_t address;
    if(host[0] >= '0' && host[0] <= '9') {
        address = inet_addr(host.c_str());
    } else {
#ifdef __GLIBC__
        char   hbuf[8192];
        struct hostent hb;
        int    rtn;
        if(gethostbyname_r(host.c_str(), &hb, hbuf, sizeof(hbuf), &hp, &rtn)) {
            hp = NULL;
        }
#else
        hp = gethostbyname(host.c_str());
#endif
        if(!hp) {
            return false;
        }
        bptr = (struct in_addr**)hp->h_addr_list;
        address = (**bptr).s_addr;
    }
    memset(&_targetAddress, 0, sizeof(struct sockaddr_in));
    _targetAddress.sin_family = AF_INET;
    _targetAddress.sin_addr.s_addr = address;
    _targetAddress.sin_port = htons(port);
    return true;
}

//-----------------------------------------------------------------------------
bool
UDPComm::_readOne(uint8_t &c)
{
    if(_inBufferCount == 0 || _inBufferIndex == _inBufferCount) {
        _inBufferCount = _read(_inBuffer, sizeof(_inBuffer));
        if(_inBufferCount <= 0) {
            return false;
        }
        _inBufferIndex = 0;
    }
    c = _inBuffer[_inBufferIndex++];
    return true;
}

//-----------------------------------------------------------------------------
int
UDPComm::_read(void* buffer, int len)
{
    struct sockaddr_in sockc;
    memset((void*)&sockc, 0, sizeof(sockc));
    socklen_t rlen = sizeof(sockc);
    while(true) {
        int count = ::recvfrom(_socket, (IOTYPE)buffer, len, 0, (struct sockaddr*)&sockc, &rlen);
        if(count == 0) {
            return 0;
        }
        if(count < 0) {
            int error = GIOERRNO;
            if(error == GEAGAIN) {
                if(!_readSelect(250)) {
                    return 0;
                }
                continue;
            }
            perror("UDP");
            _valid = false;
            return -1;
        }
        if(_switchToUnicast) {
            //-- If we were broadcasting, switch to unicast
            uint32_t a = _targetAddress.sin_addr.s_addr >> 24;
            if(a == 255) {
                _targetAddress.sin_addr = sockc.sin_addr;
            }
        }
        return count;
    }
}

//-----------------------------------------------------------------------------
int
UDPComm::_readSelect(uint32_t timeout)
{
    fd_set set;
    struct timeval tout;
    tout.tv_sec  = timeout / 1000;
    tout.tv_usec = (timeout % 1000) * 1000;
    FD_ZERO(&set);
    FD_SET(_socket, &set);
    return select(1, &set, 0, 0, &tout);
}

//-----------------------------------------------------------------------------
int
UDPComm::_writeSelect()
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(_socket, &set);
    int ret = select(_socket + 1, 0, &set, 0, 0);
    if (ret < 0) {
        return 0;
    }
    return ret != 0;
}

//-----------------------------------------------------------------------------
bool
UDPComm::_readMessage(mavlink_message_t& message)
{
    uint8_t msgReceived = 0;
    while(!msgReceived) {
        uint8_t c;
        if(_readOne(c)) {
            msgReceived = mavlink_parse_char(MAVLINK_COMM_2, c, &message, &_mstatus);
        } else {
            break;
        }
    }
    return msgReceived;
}

//-----------------------------------------------------------------------------
bool
UDPComm::_bind()
{
    struct sockaddr_in addr;
    memset((void*)&addr, 0, sizeof(addr));
    addr.sin_family         = AF_INET;
    addr.sin_addr.s_addr    = INADDR_ANY;
    addr.sin_port           = htons(MVB_UDP_LOCAL_PORT);
    if(bind(_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("UDP");
        return false;
    }
    return true;
}
