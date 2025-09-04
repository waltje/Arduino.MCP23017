/*
 * IOExpander   A library for the Arduino programming system, implementing a
 *              number of classes for use with the MCP23017 I/O Expander chip
 *		from Microchip. We support both the I2C and SPI variants.
 *
 *              This file is part of the IOExpander Library for Arduino.
 *
 *              Definitions for the MCP23017 I/O expander chip class.
 *
 * Version:     @(#)MCP23017.h 2.0.1  2025/09/02
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
#ifndef MCP23017_H
# define MCP23017_H

# include <Arduino.h>
# ifdef MCP23017_USE_SPI
#  define CLASS_NAME MCP23S17
#  include "IOExpander_SPI.h"
# else
#  define CLASS_NAME MCP23017
#  include "IOExpander_I2C.h"
# endif

/* Default address of MCP23x17. SPI uses it, too. */
# define MCP23017_ADDRESS (uint8_t)0x20


# ifdef MCP23017_USE_SPI
class CLASS_NAME : public IOExpander_SPI {
#else
class CLASS_NAME : public IOExpander_I2C {
#endif
  public:
    typedef enum PortName_e : uint8_t {
      PORT_A = 0,
      PORT_B = 1
    } PortName_t;

    /*
     * Controls if the two interrupt pins mirror each other.
     * See "3.6 Interrupt Logic".
     */
    typedef enum InterruptMode_e : uint8_t {
      IntrSeparated = 0,    ///< Interrupt pins are kept independent
      IntrOr = 0b01000000   ///< Interrupt pins are mirrored
    } InterruptMode_t;

    /*
     * Registers addresses.
     * The library use addresses for IOCON.BANK = 0.
     * See "3.2.1 Byte mode and Sequential mode".
     */
    typedef enum RegisterName_e : uint8_t {
      IODIR_A    = 0x00,    ///< Controls the direction of the data I/O for port A.
      IODIR_B    = 0x01,    ///< Controls the direction of the data I/O for port B.
      IPOL_A     = 0x02,    ///< Configures the polarity on the corresponding GPIO_ port bits for port A.
      IPOL_B     = 0x03,    ///< Configures the polarity on the corresponding GPIO_ port bits for port B.
      GPINTEN_A  = 0x04,    ///< Controls the interrupt-on-change for each pin of port A.
      GPINTEN_B  = 0x05,    ///< Controls the interrupt-on-change for each pin of port B.
      DEFVAL_A   = 0x06,    ///< Controls the default comparaison value for interrupt-on-change for port A.
      DEFVAL_B   = 0x07,    ///< Controls the default comparaison value for interrupt-on-change for port B.
      INTCON_A   = 0x08,    ///< Controls how the associated pin value is compared for the interrupt-on-change for port A.
      INTCON_B   = 0x09,    ///< Controls how the associated pin value is compared for the interrupt-on-change for port B.
      IOCON	     = 0x0a,    ///< Controls the device.
      GPPU_A     = 0x0c,    ///< Controls the pull-up resistors for the port A pins.
      GPPU_B     = 0x0d,    ///< Controls the pull-up resistors for the port B pins.
      INTF_A     = 0x0e,    ///< Reflects the interrupt condition on the port A pins.
      INTF_B     = 0x0f,    ///< Reflects the interrupt condition on the port B pins.
      INTCAP_A   = 0x10,    ///< Captures the port A value at the time the interrupt occured.
      INTCAP_B   = 0x11,    ///< Captures the port B value at the time the interrupt occured.
      GPIO_A     = 0x12,    ///< Reflects the value on the port A.
      GPIO_B     = 0x13,    ///< Reflects the value on the port B.
      OLAT_A     = 0x14,    ///< Provides access to the port A output latches.
      OLAT_B     = 0x15     ///< Provides access to the port B output latches.
    } RegisterName_t;

    typedef enum PinName_e : uint8_t {
      GPA0 = 0,
      GPA1,
      GPA2,
      GPA3,
      GPA4,
      GPA5,
      GPA6,
      GPA7,
      GPB0 = 8,
      GPB1,
      GPB2,
      GPB3,
      GPB4,
      GPB5,
      GPB6,
      GPB7
    } PinName_t;

# ifdef MCP23017_USE_SPI
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
    bool begin(uint8_t address = MCP23017_ADDRESS);

    /*
     * Controls a single pin direction. 
     * Pin 0-7 for port A, 8-15 fo port B.
     * 
     * 1 = Pin is configured as an input.
     * 0 = Pin is configured as an output.
     *
     * See "3.5.1 I/O Direction register".
     * 
     * Beware!  
     * On Arduino platform, INPUT = 0, OUTPUT = 1, which is the inverse
     * of the MCP23017 definition where a pin is an input if its IODIR bit is set to 1.
     * This library pinMode function behaves like Arduino's standard pinMode for consistency.
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
    void portMode(uint8_t port, uint8_t directions, uint8_t pullups = 0xff, uint8_t inverted = 0x00);

    /*
     * Writes a single pin state.
     * Pin 0-7 for port A, 8-15 for port B.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    void digitalWrite(uint8_t pin, uint8_t state);

    /*
     * Reads a single pin state.
     * Pin 0-7 for port A, 8-15 for port B.
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
    void writePort(uint8_t port, uint8_t value);

    /*
     * Writes pins state to both ports.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    void write(uint16_t value);

    /*
     * Reads pins state for a whole port.
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    uint8_t readPort(uint8_t port);

    /*
     * Reads pins state for both ports. 
     * 
     * 1 = Logic-high
     * 0 = Logic-low
     * 
     * See "3.5.10 Port register".
     */
    uint16_t read(void);

#if 0
    /*
     * Writes a single register value.
     */
    void writeRegister(uint8_t reg, uint8_t value);

    /*
     * Writes values to a register pair.
     * 
     * For portA and portB variable to effectively match the desired port,
     * you have to supply a portA register address to reg. Otherwise, values
     * will be reversed due to the way the MCP23017 works in Byte mode.
     */
    void writeRegister(uint8_t reg, uint8_t portA, uint8_t portB);

    /*
     * Reads a single register value.
     */
    uint8_t readRegister(uint8_t reg);

    /*
     * Reads the values from a register pair.
     * 
     * For portA and portB variable to effectively match the desired port,
     * you have to supply a portA register address to reg. Otherwise, values
     * will be reversed due to the way the MCP23017 works in Byte mode.
     */
    void readRegister(uint8_t reg, uint8_t& portA, uint8_t& portB);
#endif

#ifdef MCP23017_ENABLE_INTERRUPTS
    /*
     * Controls how the interrupt pins act with each other.
     * If intMode is Separated, interrupt conditions on a port will cause its respective INT pin to active.
     * If intMode is Or, interrupt pins are OR'ed so an interrupt on one of the port will cause both pints to active.
     * 
     * Controls the IOCON.MIRROR bit. 
     * See "3.5.6 Configuration register".
     */
    void interruptMode(InterruptMode_t intMode);

    /*
     * Configures interrupt registers using an Arduino-like API.
     * mode can be one of CHANGE, FALLING or RISING.
     */
    void interrupt(uint8_t port, uint8_t mode);

    /*
     * Disable interrupts for the specified port.
     */
    void disableInterrupt(uint8_t port);

    /*
     * Reads which pin caused the interrupt.
     */
    void interruptedBy(uint8_t& portA, uint8_t& portB);

    /*
     * Clears interrupts on both ports.
     */
    void clearInterrupts(void);

    /*
     * Clear interrupts on both ports. Returns port values at the time the interrupt occured.
     */
    void clearInterrupts(uint8_t& portA, uint8_t& portB);
#endif
};


#endif  /*MCP23017_H*/
