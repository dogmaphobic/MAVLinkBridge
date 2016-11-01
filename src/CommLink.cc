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
 * @file CommLink.cc
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "CommLink.h"

//-----------------------------------------------------------------------------
CommLink::CommLink(bool raw)
    : _rawMode(raw)
{
    memset(&_mstatus, 0, sizeof(mavlink_status_t));
}

//-----------------------------------------------------------------------------
CommLink::~CommLink()
{
    _mutex.lock();
    while(_inMessages.size()) {
        mavlink_message_t* message = _inMessages.front();
        _inMessages.pop();
        free(message);
    }
    while(_inMessagesRaw.size()) {
        std::vector<uint8_t>* data = _inMessagesRaw.front();
        _inMessagesRaw.pop();
        free(data);
    }
    _mutex.unlock();
}

//-----------------------------------------------------------------------------
mavlink_message_t*
CommLink::read()
{
    mavlink_message_t* message = NULL;
    _mutex.lock();
    if(_inMessages.size()) {
        message = _inMessages.front();
        _inMessages.pop();
    }
    _mutex.unlock();
    return message;
}

//-----------------------------------------------------------------------------
std::vector<uint8_t>*
CommLink::readRaw()
{
    std::vector<uint8_t>* data = NULL;
    _mutex.lock();
    if(_inMessagesRaw.size()) {
        data = _inMessagesRaw.front();
        _inMessagesRaw.pop();
    }
    _mutex.unlock();
    return data;
}

