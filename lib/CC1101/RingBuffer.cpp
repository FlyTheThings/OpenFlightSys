/*
 * mbed library for RingBuffer
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */

#include "RingBuffer.h"

RingBuffer::RingBuffer (int p_size) {
    size = p_size + 1;
    buf = new char[size];
    addr_w = 0;
    addr_r = 0;
}
/*
RingBuffer::~RingBuffer () {
    delete [] buf;
}
*/
int RingBuffer::_putc (int dat) {
    int next;

    next = (addr_w + 1) % size;
    if (next == addr_r) {
        return -1;
    }
    buf[addr_w] = dat;
    addr_w = next;
    return dat;
}

int RingBuffer::put (char *dat, int len) {
    int next, i;


    for (i = 0; i < len; i ++) {
        next = (addr_w + 1) % size;
        if (next == addr_r) {
            break;
        }
        buf[addr_w] = dat[i];
        addr_w = next;
    }
    return i;
}

int RingBuffer::_getc (void) 
{
    char dat;
    if (addr_r == addr_w) {
        return 0;
    }
    dat = buf[addr_r];
    addr_r = (addr_r + 1) % size;
    return dat;
}

int RingBuffer::get (char *dat, int len) {
    int i;

    for (i = 0; i < len; i ++) {
        if (addr_r == addr_w) {
            break;
        }
        dat[i] = buf[addr_r];
        addr_r = (addr_r + 1) % size;
    }
    return i;
}

int RingBuffer::available () {
    if (addr_w < addr_r) {
        return addr_r - addr_w - 1;
    } else {
        return (size - addr_w) + addr_r - 1;
    }
}

int RingBuffer::use () {
    return size - available() - 1;
}

void RingBuffer::clear () {
    addr_w = 0;
    addr_r = 0;
}
