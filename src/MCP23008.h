/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Definitions for the MCP23008 I/O expander chip class.
 *
 * Version:     @(#)MCP23000.h 1.0.2  2026/01/15
 *
 * Authors:     Fred N. van Kempen, <decwiz@yahoo.com>
 *              Bertrand Lemasle, <https://github.com/blemasle>
 *
 *              Copyright 2024-2026 MicroWalt Corporation LLC.
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
#ifndef MCP23008_H
# define MCP23008_H

# include <Arduino.h>
# ifdef CLASS_NAME
#  undef CLASS_NAME
# endif
# ifdef MCP23008_USE_SPI
#  define CLASS_NAME MCP23S08
#  include "IOExpander_SPI.h"
# else
#  define CLASS_NAME MCP23008
#  include "IOExpander_I2C.h"
# endif

/* Default address of MCP23x08. SPI uses it, too. */
# define MCP23008_ADDRESS (uint8_t)0x20


# ifdef MCP23008_USE_SPI
class CLASS_NAME : public IOExpander_SPI {
#else
class CLASS_NAME : public IOExpander_I2C {
#endif
  public:
    /*
     * Registers addresses.
     * The library use addresses for IOCON.BANK = 0.
     * See "3.2.1 Byte mode and Sequential mode".
     */
    typedef enum RegisterName_e : uint8_t {
      IODIR    = 0x00,    ///< Controls the direction of the data I/O for the port.
      IPOL     = 0x01,    ///< Configures the polarity on the corresponding GPIO_ port bits.
      GPINTEN  = 0x02,    ///< Controls the interrupt-on-change for each pin of the port.
      DEFVAL   = 0x03,    ///< Controls the default comparaison value for interrupt-on-change for the port.
      INTCON   = 0x04,    ///< Controls how the associated pin value is compared for the interrupt-on-change for the port.
      IOCON    = 0x05,    ///< Controls the device.
      GPPU     = 0x06,    ///< Controls the pull-up resistors for the port pins.
      INTF     = 0x07,    ///< Reflects the interrupt condition on the port pins.
      INTCAP   = 0x08,    ///< Captures the port value at the time the interrupt occured.
      GPIO     = 0x09,    ///< Reflects the value on the port.
      OLAT     = 0x0a     ///< Provides access to the port output latches.
    } RegisterName_t;

    typedef enum PinName_e : uint8_t {
      GPA0 = 0,
      GPA1,
      GPA2,
      GPA3,
      GPA4,
      GPA5,
      GPA6,
      GPA7
    } PinName_t;

# ifdef MCP23008_USE_SPI
    CLASS_NAME(SPIClass *spi, int ss = SS);
    CLASS_NAME(SPIClass& spi, int ss = SS);
# else
    CLASS_NAME(TwoWire *theWire);
    CLASS_NAME(TwoWire& theWire);
# endif
    CLASS_NAME();

    /*
     * Start up the library.
     *
     * You can set SPI parameters using the setSPIxxx functions,
     * and likewise, for I2C, with the setWirexxx functions.
     */
    bool begin(uint8_t address = MCP23008_ADDRESS);

    /*
     * Controls a single pin direction. 
     * Pin 0-7 for port A.
     * 
     * 1 = Pin is configured as an input.
     * 0 = Pin is configured as an output.
     *
     * See "3.5.1 I/O Direction register".
     * 
     * Beware!  
     * On Arduino platform, INPUT = 0, OUTPUT = 1, which is the inverse
     * of the MCP23017 definition where a pin is an input if its IODIR
     * bit is set to 1. This library pinMode function behaves like the
     * Arduino standard pinMode for consistency.
     * [ OUTPUT | INPUT | INPUT_PULLUP ]
     */
    void pinMode(uint8_t pin, uint8_t mode, bool inverted = false);

    /*
     * Controls the pins direction on a whole port at once.
     * 
     * 1 = Pin is configured as an input.
     * 0 = Pin is configured as an output.
     * 
     * See "3.5.1 I/O Direction register".
     */
    void portMode(uint8_t directions, uint8_t pullups = 0xff, uint8_t inverted = 0x00);

    /*
     * Writes a single pin state.
     * Pin 0-7 for port A.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    void digitalWrite(uint8_t pin, uint8_t state);

    /*
     * Reads a single pin state.
     * Pin 0-7 for port A.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */ 
    uint8_t digitalRead(uint8_t pin);

    /*
     * Writes pins state to a whole port.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    void writePort(uint8_t value);

    /*
     * Writes pins state to both ports.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    void write(uint8_t value);

    /*
     * Reads pins state for a whole port.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    uint8_t readPort(void);

    /*
     * Reads pins state for both ports. 
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    uint8_t read(void);

    /*
     * Configures interrupt registers using an Arduino-like API.
     * mode can be one of CHANGE, FALLING or RISING.
     */
    void interrupt(uint8_t mode);

    /*
     * Disable interrupts for the port.
     */
    void disableInterrupt(void);

    /*
     * Reads which pin caused the interrupt.
     */
    void interruptedBy(uint8_t *portA);

    /*
     * Clears interrupts on the port.
     */
    void clearInterrupts(void);

    /*
     * Clear interrupts on the port.
     * Returns port value at the time the interrupt occured.
     */
    void clearInterrupts(uint8_t *portA);
};


#endif  /*MCP23008_H*/
