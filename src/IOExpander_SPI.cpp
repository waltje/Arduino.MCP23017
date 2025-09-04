/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Interface class for devices connected through SPI.
 *
 * Version:     @(#)IOExpander_SPI.cpp 2.0.1  2025/09/02
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
#include "IOExpander_SPI.h"


/* This seems to be the common default. */
#define SPI_FREQUENCY   1E6 

// Opcode for MCP23S17 with LSB (bit0) set to write (0),
// address OR'd in later, bits 1-3
#define OPCODE_W (0b01000000)

// Opcode for MCP23S17 with LSB (bit0) set to read (1),
// address OR'd in later, bits 1-3
#define OPCODE_R (0b01000001)


IOExpander_SPI::IOExpander_SPI() :
  _spi(NULL),
  _speed(SPI_FREQUENCY),
  _spiSettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _address(0xff),
  _ss(-1),
  _intr(-1)
{
}


void
IOExpander_SPI::setSPI(SPIClass *spi)
{
  _spi = spi;
}


void
IOExpander_SPI::setSPI(SPIClass& spi)
{
  _spi = &spi;
}


void
IOExpander_SPI::setSPIFrequency(uint32_t frequency)
{
  _speed = frequency;
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}


void
IOExpander_SPI::setPins(int ss, int intr)
{
  if (ss != -1)
    _ss = (int8_t)ss;

  if (intr != -1)
    _intr = (int8_t)intr;
}


bool
IOExpander_SPI::reset(uint8_t address)
{
  /* Set the address. */
  _address = address;

  /* Set up the SS pin. */
  ::pinMode(_ss, OUTPUT);
  ::digitalWrite(_ss, HIGH);

  /* Start or re-start the SPI controller. */
  _spi->begin();

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
IOExpander_SPI::end(void)
{
  if (_intr != -1) {
    detachInterrupt(digitalPinToInterrupt(_intr));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_intr));
#endif
  }

  /* Stop the SPI bus. */
  _spi->end();
}


uint8_t
IOExpander_SPI::readRegister(uint8_t reg)
{
  uint8_t val;

  ::digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  (void)_spi->transfer((_address << 1) | OPCODE_R);
  (void)_spi->transfer(reg);
  val = _spi->transfer(0x00);
  _spi->endTransaction();

  ::digitalWrite(_ss, HIGH);

  return val;
}


void
IOExpander_SPI::readRegister(uint8_t reg, uint8_t& portA, uint8_t& portB)
{
  ::digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  (void)_spi->transfer((_address << 1) | OPCODE_R);
  (void)_spi->transfer(reg);
  portA = _spi->transfer(0x00);
  portB = _spi->transfer(0x00);
  _spi->endTransaction();

  ::digitalWrite(_ss, HIGH);
}


void
IOExpander_SPI::writeRegister(uint8_t reg, uint8_t val)
{
  ::digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  (void)_spi->transfer((_address << 1) | OPCODE_W);
  (void)_spi->transfer(reg);
  (void)_spi->transfer(val);
  _spi->endTransaction();

  ::digitalWrite(_ss, HIGH);
}


void
IOExpander_SPI::writeRegister(uint8_t reg, uint8_t portA, uint8_t portB)
{
  ::digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  (void)_spi->transfer((_address << 1) | OPCODE_W);
  (void)_spi->transfer(reg);
  (void)_spi->transfer(portA);
  (void)_spi->transfer(portB);
  _spi->endTransaction();

  ::digitalWrite(_ss, HIGH);
}
