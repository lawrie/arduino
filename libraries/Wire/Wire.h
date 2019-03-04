/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>
#include "Arduino.h"

#define BUFFER_LENGTH 32

#define I2C_TX_DATA (*(volatile uint32_t*)0xF0070000)
#define I2C_TX_ACK (*(volatile uint32_t*)0xF0070004)
#define I2C_RX_DATA (*(volatile uint32_t*)0xF0070008)
#define I2C_RX_ACK (*(volatile uint32_t*)0xF007000C)
#define I2C_INTERRUPT (*(volatile uint32_t*)0xF0070020)
#define I2C_SAMPLING_CLOCK_DIVIDER (*(volatile uint32_t*)0xF0070028)
#define I2C_TIMEOUT (*(volatile uint32_t*)0xF007002C)
#define I2C_TSU_DAT (*(volatile uint32_t*)0xF0070030)
#define I2C_MASTER_STATUS (*(volatile uint32_t*)0xF0070040)
#define I2C_TLOW (*(volatile uint32_t*)0xF0070050)
#define I2C_THIGH (*(volatile uint32_t*)0xF0070054)
#define I2C_TBUF (*(volatile uint32_t*)0xF0070058)
#define I2C_FILTER_STATUS (*(volatile uint32_t*)0xF0070080)
#define I2C_HIT_CONTEXT (*(volatile uint32_t*)0xF0070084)
#define I2C_FILTERING_CONFIG (*(volatile uint32_t*)0xF0070088)


#define I2C_MASTER_IS_BUSY (1 << 0)
#define I2C_MASTER_START (1 << 4)
#define I2C_MASTER_STOP (1 << 5)

#define I2C_TX_ENABLE (1 << 9)
#define I2C_TX_VALID (1 << 8)

#define I2C_RX_ENABLE (1 << 9)
#define I2C_RX_VALID (1 << 8)
#define I2C_RX_LISTEN (1 << 9)

#define I2C_MAX_READ 10

#define MAX_TRIES 1000

class TwoWire : public Stream
{
  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static uint8_t transmitting;
    static void onRequestService(void);
    static void onReceiveService(uint8_t*, int);
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(int);
    static void sda_rising_isr(void);
  public:
    TwoWire();
    void begin();
    void begin(uint8_t);
    void begin(int);
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
	virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );
  
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;
};

extern TwoWire Wire;
#endif

