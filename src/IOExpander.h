/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Definitions for the API part of the library.
 *
 * Version:     @(#)IOExpander.h 2.0.1  2025/09/02
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
#ifndef IOEXPANDER_H
# define IOEXPANDER_H

#include <Arduino.h>


class IOExpander {
  public:
    IOExpander();

    /*
     * Controls a single pin direction. 
     * Pin 0-7 for port A, 8-15 fo port B.
     * 
     * 1 = Pin is configured as an input.
     * 0 = Pin is configured as an output.
     *
     * BEWARE!  
     * On the Arduino platform, INPUT=0, OUTPUT=1, which is the inverse
     * of the MCP23017 definition where a pin is an input if its IODIR
     * bit is set to 1. This library pinMode function behaves like the
     * Arduino standard pinMode for consistency:
     *
     *   [ OUTPUT | INPUT | INPUT_PULLUP ]
     */
    virtual void pinMode(uint8_t pin, uint8_t mode, bool inverted = false) = 0;

    /*
     * Controls the pins direction on a whole port at once.
     * 
     * 1 = Pin is configured as an input.
     * 0 = Pin is configured as an output.
     */
    virtual void portMode(uint8_t port, uint8_t directions, uint8_t pullups = 0xff, uint8_t inverted = 0x00) = 0;

    /*
     * Writes a single pin state.
     * Pin 0-7 for port A, 8-15 for port B.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */
    virtual void digitalWrite(uint8_t pin, uint8_t state) = 0;

    /*
     * Reads a single pin state.
     * Pin 0-7 for port A, 8-15 for port B.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */ 
    virtual uint8_t digitalRead(uint8_t pin) = 0;

    /*
     * Writes pins state to a whole port.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */
    virtual void writePort(uint8_t port, uint8_t value) = 0;

    /*
     * Writes pins state to both ports.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */
    virtual void write(uint16_t value) = 0;

    /*
     * Reads pins state for a whole port.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */
    virtual uint8_t readPort(uint8_t port) = 0;

    /*
     * Reads pins state for both ports. 
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     */
    virtual uint16_t read(void) = 0;

#ifdef _IOExpander_Enable_Interrupts
    /*
     * Configures interrupt registers using an Arduino-like API.
     * mode can be one of CHANGE, FALLING or RISING.
     */
    virtual void interrupt(uint8_t port, uint8_t mode) = 0;

    /*
     * Disable interrupts for the specified port.
     */
    virtual void disableInterrupt(uint8_t port) = 0;

    /*
     * Reads which pin caused the interrupt.
     */
    virtual void interruptedBy(uint8_t& portA, uint8_t& portB) = 0;

    /*
     * Clears interrupts on both ports.
     */
    virtual void clearInterrupts(void) = 0;

    /*
     * Clear interrupts on both ports.
     * Returns port values at the time the interrupt occured.
     */
    virtual void clearInterrupts(uint8_t& portA, uint8_t& portB) = 0;
#endif

    /*
     * Sometimes an application wants to read registers...
     */
    virtual uint8_t readRegister(uint8_t reg) = 0;
    virtual void readRegister(uint8_t reg, uint8_t& portA, uint8_t& portB) = 0;
    virtual void writeRegister(uint8_t reg, uint8_t val) = 0;
    virtual void writeRegister(uint8_t reg, uint8_t portA, uint8_t portB) = 0;

  protected:
    bool _valid;

  private:
};


#endif  /*IOEXPANDER_H*/
