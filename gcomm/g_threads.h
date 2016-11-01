/****************************************************************************
 *
 * Copyright (c) 1991, 2016 Gus Grubba. All rights reserved.
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
 * @file g_threads.h
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */


#ifndef _G_THREADS_H_
#define _G_THREADS_H_

#include "g_platform.h"
#include <pthread.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

//-----------------------------------------------------------------------------
class gEvent
{
public:
    explicit gEvent()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_VALUE);
        pthread_mutex_init(&_mutex, &attr);
        pthread_mutexattr_destroy(&attr);
        pthread_condattr_t cattr;
        pthread_condattr_init(&cattr);
        pthread_cond_init(&_cond, &cattr);
        pthread_condattr_destroy(&cattr);
    }
    //---------------------------------------------------------------------
    ~gEvent()
    {
        pthread_cond_destroy(&_cond);
        pthread_mutex_destroy(&_mutex);
    }
    //---------------------------------------------------------------------
    void set()
    {
        pthread_cond_signal(&_cond);
    }
    //---------------------------------------------------------------------
    bool wait(int waitTime)
    {
        struct timeval tp;
        gettimeofday(&tp, NULL);
        struct timespec ts;
        ts.tv_sec = tp.tv_sec;
        ts.tv_nsec = tp.tv_usec * 1000;
        int t = waitTime;
        while(t > 1000) {
            ts.tv_sec++;
            t -= 1000;
        }
        ts.tv_nsec += (t * 1000);
        return pthread_cond_timedwait(&_cond, &_mutex, &ts) != ETIMEDOUT;
    }
private:
    pthread_mutex_t     _mutex;
    pthread_cond_t      _cond;
};

//-----------------------------------------------------------------------------
class gMutex
{
public:
    //---------------------------------------------------------------------
    gMutex()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_VALUE);
        pthread_mutex_init(&_mutex, &attr);
        pthread_mutexattr_destroy(&attr);
    }
    //---------------------------------------------------------------------
    ~gMutex()
    {
        pthread_mutex_destroy(&_mutex);
    }
    //---------------------------------------------------------------------
    void lock()
    {
        pthread_mutex_lock(&_mutex);
    }
    //---------------------------------------------------------------------
    void unlock()
    {
        pthread_mutex_unlock(&_mutex);
    }
private:
    pthread_mutex_t _mutex;
};

//-----------------------------------------------------------------------------
class gThread
{
public:
    enum {
        THREAD_NEW,
        THREAD_STARTED,
        THREAD_RUNNING,
        THREAD_ENDED,
        THREAD_ERROR
    };
    //---------------------------------------------------------------------
    gThread()
    {
        _status         = THREAD_NEW;
        _tid            = 0;
        pthread_attr_init(&_attr);
        pthread_attr_setdetachstate(&_attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setinheritsched(&_attr, PTHREAD_INHERIT_SCHED);
    }
    //---------------------------------------------------------------------
    virtual ~gThread()
    {
    }
    //---------------------------------------------------------------------
    virtual void run() = 0;
    //---------------------------------------------------------------------
    virtual void join()
    {
        pthread_join(_tid, 0);
    }
    //---------------------------------------------------------------------
    virtual bool startThread()
    {
        if(_status != THREAD_NEW) {
            return false;
        }
        _status = THREAD_STARTED;
        typedef void* (*exec_t)(void*);
        if(pthread_create(&_tid, &_attr, exec_t(&_execute), this) == 0) {
            return true;
        }
        _status = THREAD_ERROR;
        return false;
    }
    //---------------------------------------------------------------------
    bool threadActive()
    {
        return _status == THREAD_STARTED || _status == THREAD_RUNNING;
    }
    //---------------------------------------------------------------------
    void setThreadStatus(int status)
    {
        _status = status;
    }
private:
    //---------------------------------------------------------------------
    static void _execute(gThread* th)
    {
        th->setThreadStatus(THREAD_RUNNING);
        th->run();
        th->setThreadStatus(THREAD_ENDED);
    }
    int             _status;
    pthread_t       _tid;
    pthread_attr_t  _attr;
};

#endif
