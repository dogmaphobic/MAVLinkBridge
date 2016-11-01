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
 * @file UDPComm.h
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef UDPCOMM_H
#define UDPCOMM_H

#define MVB_UDP_TARGET_PORT 14550
#define MVB_UDP_LOCAL_PORT  14555

#include "CommLink.h"

class UDPComm : public CommLink
{
public:
    UDPComm(bool raw, bool switchToUnicast);
    ~UDPComm();
    bool            init        (std::string address);
    bool            open        ();
    void            close       ();
    void            write       (mavlink_message_t& message);
    void            write       (std::vector<uint8_t> data);
    //-- From gThread
    void            run();
private:
    bool            _sendTo     (void *buffer, int len);
    bool            _setTargetAddress (const std::string& host, uint16_t port);
    bool            _readOne    (uint8_t& c);
    int             _read       (void* buffer, int len);
    int             _readSelect (uint32_t timeout);
    int             _writeSelect();
    bool            _readMessage(mavlink_message_t& message);
    bool            _bind       ();
private:
    SOCKET              _socket;
    struct sockaddr_in  _targetAddress;
    bool                _valid;
    bool                _switchToUnicast;
    uint8_t             _inBuffer[128];
    uint8_t             _inBufferIndex;
    uint8_t             _inBufferCount;
};

#endif
