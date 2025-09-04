/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Interface class for devices connected through I2C.
 *
 * Version:     @(#)IOExpander_I2C.cpp 2.0.1  2025/09/02
 *
 * Author:      Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *              Copyright 2024,2025 MicroWalt Corporation LLC.
 *
 *              Redistribution and  use  in source  and binary forms, with
 *              or  without modification, are permitted  provided that the
 *              following conditions are met:
 *
 *              1. Redistributions of  source  code must retain the entire
 *                 above notice, this list of conditions and the following
 *                 disclaimer.
 *
 *              2. Redistributions in binary form must reproduce the above
 *                 copyright  notice,  this list  of  conditions  and  the
 *                 following disclaimer in  the documentation and/or other
 *                 materials provided with the distribution.
 *
 *              3. Neither the  name of the copyright holder nor the names
 *                 of  its  contributors may be used to endorse or promote
 *                 products  derived from  this  software without specific
 *                 prior written permission.
 *
 * THIS SOFTWARE  IS  PROVIDED BY THE  COPYRIGHT  HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING, BUT  NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE  ARE  DISCLAIMED. IN  NO  EVENT  SHALL THE COPYRIGHT
 * HOLDER OR  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 * THEORY OF  LIABILITY, WHETHER IN  CONTRACT, STRICT  LIABILITY, OR  TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  IN ANY  WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "IOExpander_I2C.h"


/* This seems to be the common default. */
#define I2C_FREQUENCY   100000


IOExpander_I2C::IOExpander_I2C() :
  _wire(NULL),
  _speed(I2C_FREQUENCY),
  _address(0xff),
  _intr(-1)
{
}


void
IOExpander_I2C::setWire(TwoWire *theWire)
{
  _wire = theWire;
}


void
IOExpander_I2C::setWire(TwoWire& theWire)
{
  _wire = &theWire;
}


void
IOExpander_I2C::setWireFrequency(uint32_t frequency)
{
  _speed = frequency;
}


void
IOExpander_I2C::setPins(int intr)
{
  _intr = (int8_t)intr;
}


bool
IOExpander_I2C::reset(uint8_t address)
{
  /* Save the device address. */
  _address = address;

  /* Start or re-start the I2C controller. */
  _wire->begin();

  /* See if we can talk to the chip. */
  _wire->beginTransmission(_address);
  if (_wire->endTransmission())
    return false;

  /* Set up the INTR pin. */
  if (_intr != -1) {
    pinMode(_intr, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_intr));
#endif
//  attachInterrupt(digitalPinToInterrupt(_intr), __isr, RISING);
  }

  return true;
}


void
IOExpander_I2C::end(void)
{
  if (_intr != -1) {
    detachInterrupt(digitalPinToInterrupt(_intr));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_intr));
#endif
  }

  /* Stop the I2C bus. */
#ifndef ARDUINO_ARCH_ESP8266
  _wire->end();
#endif
}


uint8_t
IOExpander_I2C::readRegister(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_address, (uint8_t)1);

  return _wire->read();
}


void
IOExpander_I2C::readRegister(uint8_t reg, uint8_t& portA, uint8_t& portB)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_address, (uint8_t)2);

  portA = _wire->read();
  portB = _wire->read();
}


void
IOExpander_I2C::writeRegister(uint8_t reg, uint8_t val)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}


void
IOExpander_I2C::writeRegister(uint8_t reg, uint8_t portA, uint8_t portB)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(portA);
  _wire->write(portB);
  _wire->endTransmission();
}
