/**
 * DmxSimple - A simple interface to DMX.
 *
 * Copyright (c) 2008-2009 Peter Knight, Tinker.it! All rights reserved.
 */

#ifndef DmxSimple_h
#define DmxSimple_h

#include <inttypes.h>

#define DMX_SIZE 8
//neni pouzivan timer2 a volano jeho preruseni 
#define NO_TIMER_CALL


class DmxSimpleClass
{
  public:
    void maxChannel(int);
    void write(int, uint8_t);
    void usePin(uint8_t);
    void sendData();
};
extern DmxSimpleClass DmxSimple;

#endif
