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
 * @file g_platform.h
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef _G_PLATFORM_H_
#define _G_PLATFORM_H_

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#ifdef __APPLE__
#include <netinet/in_systm.h>
#include <fnmatch.h>
#include <sys/param.h>
#include <sys/mount.h>
#include <sys/sockio.h>
#endif
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

//-----------------------------------------------
//-- Unix

#define g_fseek64       fseeko
#define g_ftell64       ftello

#define INVALID_SOCKET  -1
#define SOCKET          int
#define SOCKOPTVAL
#define IOTYPE          void *
#define GIOERRNO        errno
#define GEAGAIN         EAGAIN
#define GEADDRINUSE     EADDRINUSE
#define GECONNRESET     ECONNRESET
#define GECONNREFUSED   ECONNREFUSED
#define GSNOTTHERE      EADDRNOTAVAIL
#define _CDECL
#define GSOCADDRLEN     socklen_t

#define g_alloc(s)      malloc(s)
#define g_malloc(s)     malloc(s)
#define g_free          free
#define g_sleep(c)      usleep((c*1000))
#define g_ioctl(a,b,c)  ioctl(a,b,c)

#if defined(__sgi)
#define g_size_t        int
#elif defined(__APPLE__)
#define g_size_t        socklen_t
#else
#define g_size_t        size_t
#endif
#define g_chmod         chmod
#define g_chdir         chdir

#define g_vsnprintf     vsnprintf

#define _DIRSLASH       _T("/")
#define _CDIRSLASH      _T('/')
#define _WILDFILE       _T("*")
#define _PATHDELIMITER  _T(":")

#define INVALID_HANDLE_VALUE -1

extern void     g_sleepimp(uint32_t msec);

#ifndef g_max
#define g_max(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef g_min
#define g_min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifdef __APPLE__
#define PTHREAD_MUTEX_RECURSIVE_VALUE PTHREAD_MUTEX_RECURSIVE
#else
#define PTHREAD_MUTEX_RECURSIVE_VALUE PTHREAD_MUTEX_RECURSIVE_NP
#endif

#endif
