/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Driver class for the Microchip MCP23017 I/O expander chip.
 *
 * NOTE         To avoid duplicating lots of code, we use some tricks with
 *              macros. The "default" version of this file will compile for
 *              use with I2C.  If MCP23017_USE_SPI is defined, it will do
 *              a compile for use with SPI instead.  We have MCP23S17 .cpp
 *              and .h files which just set this macro, and then include
 *              this file (and its .h file.)
 *
 * Version:     @(#)MCP23017.cpp 2.0.1  2025/09/02
 *
 * Authors:     Fred N. van Kempen, <decwiz@yahoo.com>
 *              Bertrand Lemasle, <https://github.com/blemasle>
 *
 *              Copyright 2024,2025 MicroWalt Corporation LLC.
 *              Copyright 2017-2024 Bertrand Lemasle.
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
#include "MCP23017.h"


#ifdef MCP23017_USE_SPI
CLASS_NAME::CLASS_NAME(SPIClass *spi, int ss)
{
  setSPI(spi);
  setPins(ss);
}


CLASS_NAME::CLASS_NAME(SPIClass& spi, int ss)
{
  setSPI(&spi);
  setPins(ss);
}
#else
CLASS_NAME::CLASS_NAME(TwoWire *theWire)
{
  setWire(theWire);
}


CLASS_NAME::CLASS_NAME(TwoWire& theWire)
{
  setWire(&theWire);
}
#endif


CLASS_NAME::CLASS_NAME()
{
  _valid = false;
}


bool
CLASS_NAME::begin(uint8_t address)
{
  /* Initialyze the interface driver. */
  _valid = reset(address);

  if (_valid) {
    /*
     * Write the I/O configuration register:
     *
     * BANK         = 0 : sequential register addresses
     * MIRROR       = 0 : use configureInterrupt
     * SEQOP        = 1 : sequential operation disabled, address pointer does not increment
     * DISSLW       = 0 : slew rate enabled
     * HAEN         = 0/1 : hardware address (1 disables it)
     * ODR          = 0 : active driver output (INTPOL bit sets the polarity.)
     * INTPOL       = 0 : interrupt active low
     * UNIMPLMENTED   0 : unimplemented: Read as ‘0’
     */
#ifdef MCP23017_USE_SPI
    writeRegister(CLASS_NAME::IOCON, 0b00101000);
#else
    writeRegister(CLASS_NAME::IOCON, 0b00100000);
#endif
  }

  return _valid;
}


void
CLASS_NAME::pinMode(uint8_t pin, uint8_t mode, bool inverted)
{
  uint8_t iodirreg = CLASS_NAME::IODIR_A;
  uint8_t pullupreg = CLASS_NAME::GPPU_A;
  uint8_t polreg = CLASS_NAME::IPOL_A;
  uint8_t iodir, pol, pull;

  if (pin > 7) {
    iodirreg = CLASS_NAME::IODIR_B;
    pullupreg = CLASS_NAME::GPPU_B;
    polreg = CLASS_NAME::IPOL_B;
    pin -= 8;
  }

  iodir = readRegister(iodirreg);
  if (mode == INPUT || mode == INPUT_PULLUP)
    bitSet(iodir, pin);
  else
    bitClear(iodir, pin);

  pull = readRegister(pullupreg);
  if (mode == INPUT_PULLUP)
    bitSet(pull, pin);
  else
    bitClear(pull, pin);

  pol = readRegister(polreg);
  if (inverted)
    bitSet(pol, pin);
  else
    bitClear(pol, pin);

  writeRegister(iodirreg, iodir);
  writeRegister(pullupreg, pull);
  writeRegister(polreg, pol);
}


void
CLASS_NAME::portMode(uint8_t port, uint8_t directions, uint8_t pullups, uint8_t inverted)
{
  writeRegister(CLASS_NAME::IODIR_A + port, directions);
  writeRegister(CLASS_NAME::GPPU_A + port, pullups);
  writeRegister(CLASS_NAME::IPOL_A + port, inverted);
}


void
CLASS_NAME::digitalWrite(uint8_t pin, uint8_t state)
{
  uint8_t gpioreg = CLASS_NAME::GPIO_A;
  uint8_t gpio;

  if (pin > 7) {
    gpioreg = CLASS_NAME::GPIO_B;
    pin -= 8;
  }

  gpio = readRegister(gpioreg);
  if (state == HIGH)
    bitSet(gpio, pin);
  else
    bitClear(gpio, pin);

  writeRegister(gpioreg, gpio);
}


uint8_t
CLASS_NAME::digitalRead(uint8_t pin)
{
  uint8_t gpioreg = CLASS_NAME::GPIO_A;
  uint8_t gpio;

  if (pin > 7) {
    gpioreg = CLASS_NAME::GPIO_B;
    pin -=8;
  }

  gpio = readRegister(gpioreg);

  if (bitRead(gpio, pin))
    return HIGH;

  return LOW;
}


void
CLASS_NAME::writePort(uint8_t port, uint8_t value)
{
  writeRegister(CLASS_NAME::GPIO_A + port, value);
}


void
CLASS_NAME::write(uint16_t value)
{
  writeRegister(CLASS_NAME::GPIO_A, lowByte(value), highByte(value));
}


uint8_t
CLASS_NAME::readPort(uint8_t port)
{
  return readRegister(CLASS_NAME::GPIO_A + port);
}


uint16_t
CLASS_NAME::read(void)
{
  uint8_t a = readPort(CLASS_NAME::PORT_A);
  uint8_t b = readPort(CLASS_NAME::PORT_B);

  return a | b << 8;
}


#ifdef MCP23017_ENABLE_INTERRUPTS
void
CLASS_NAME::interruptMode(CLASS_NAME::InterruptMode_t intMode)
{
  uint8_t iocon = readRegister(CLASS_NAME::IOCON);

  if (intMode == CLASS_NAME::Or)
    iocon |= static_cast<uint8_t>(CLASS_NAME::Or);
  else
    iocon &= ~(static_cast<uint8_t>(CLASS_NAME::Or));

  writeRegister(CLASS_NAME::IOCON, iocon);
}


void
CLASS_NAME::interrupt(uint8_t port, uint8_t mode)
{
  uint8_t defvalreg = CLASS_NAME::DEFVAL_A + port;
  uint8_t intconreg = CLASS_NAME::INTCON_A + port;

  /* Enable interrupt for port. */
  writeRegister(CLASS_NAMECP::GPINTEN_A + port, 0xff);
  switch (mode) {
    case CHANGE:        // interrupt on change
      writeRegister(intconreg, 0);
      break;

    case FALLING:       // interrupt falling : compared against defval, 0xff
      writeRegister(intconreg, 0xff);
      writeRegister(defvalreg, 0xff);
      break;

    case RISING:        // interrupt rising : compared against defval, 0x00
      writeRegister(intconreg, 0xff);
      writeRegister(defvalreg, 0x00);
      break;
  }
}


void
CLASS_NAME::interruptedBy(uint8_t& portA, uint8_t& portB)
{
  readRegister(CLASS_NAME::INTF_A, portA, portB);
}


void
CLASS_NAME::disableInterrupt(uint8_t port)
{
  writeRegister(CLASS_NAME::GPINTEN_A + port, 0x00);
}


void
CLASS_NAME::clearInterrupts(uint8_t& portA, uint8_t& portB)
{
  readRegister(CLASS_NAME::INTCAP_A, portA, portB);
}


void
CLASS_NAME::clearInterrupts(void)
{
  uint8_t a, b;

  clearInterrupts(a, b);
}
#endif
