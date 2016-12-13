/*
 * mbed library for RingBuffer
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */

#ifndef RingBuffer_H
#define RingBuffer_H

#include "mbed.h"

class RingBuffer : public Stream  {
public:
    RingBuffer (int p_size);
//    ~RingBuffer ();

#if DOXYGEN_ONLY
    int putc(int c);
    int printf(const char* format, ...);
#endif
//    int putc (char);
    int put (char *, int);
    char get ();
    int get (char *, int);
    void clear ();
    int available ();
    int use ();

private:
    // Stream implementation functions
    virtual int _putc(int value);
    virtual int _getc();

    char *buf;
    int size;
    int addr_w, addr_r;
};

#endif
