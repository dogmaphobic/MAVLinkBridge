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
 * @file CommLink.h
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef COMMLINK_H
#define COMMLINK_H

#include "g_platform.h"
#include "g_threads.h"
#include "mavlink.h"

#include <string>
#include <queue>

class CommLink : public gThread
{
public:
    CommLink(bool raw);
    virtual ~CommLink();
    virtual bool    open()=0;
    virtual void    close()=0;
    virtual void    write(mavlink_message_t& message)=0;
    virtual void    write(std::vector<uint8_t> data)=0;
    //-- Caller owns pointer
    virtual mavlink_message_t*      read();
    virtual std::vector<uint8_t>*   readRaw();
protected:
    gMutex _mutex;
    std::queue<mavlink_message_t*>      _inMessages;
    std::queue<std::vector<uint8_t>*>   _inMessagesRaw;
    mavlink_status_t                    _mstatus;
    bool _rawMode;
};

#endif
