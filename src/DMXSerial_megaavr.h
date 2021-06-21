// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_magaavr.h: Hardware specific functions for MEGAAVR processors like 4809 used in Arduino Every.

// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -

// global variables and functions are prefixed with "_DMX_"

// ----- MegaAVR specific Hardware abstraction functions -----

#ifndef DMXSERIAL_MEGAAVR_H
#define DMXSERIAL_MEGAAVR_H

#if defined(DMXFORMAT) && defined(ARDUINO_ARCH_MEGAAVR)

#include "Arduino.h"
#include "DMXSerial.h"
#include "avr/io.h"

/* To support other boards you must define the pins here 
 * USARTNUM is the number of the USART to use, 0 - 3
 * USARTMUXOPT is the constant like PORTMUX_USART1_ALT1_gc
 * USARTRX_PIN and USARTTX_PIN are the RX and TX pins of that USART. 
 * The commented out example is for an ATtiny0/1/2-series with at least 14 pins
 */

#define _DMX_USARTNUM 0
#define _DMX_USARTMUXOPT PORTMUX_USART0_DEFAULT_gc
#define _DMX_USARTRX_PIN 3
#define _DMX_USARTTX_PIN 2
#define _DMX_USARTMUXOPT PORTMUX_USART1_ALT1_gc


/* Nothing below here should need to be changed by the user */
#ifndef _DMX_USARTNUM
  #ifndef (__AVR_ATmega4809__)
    #error "To use this library with non-4809 modern AVRs, you must add the appropriate definitions to DMXSerial_megaavr.h"
  #endif
  #define _DMX_USARTNUM 1
  #define _DMX_USARTMUXOPT PORTMUX_USART1_ALT1_gc
  #define _DMX_USARTRX_PIN PIN_WIRE_HWSERIAL1_RX
  #define _DMX_USARTTX_PIN PIN_WIRE_HWSERIAL1_TX
#endif

#if (((defined(USARTNUM) && USARTNUM == 0) || !defined(USARTNUM)) && defined(USART0))
  #define _DMX_USART USART0
  #define _DMX_USART_RXC_vect USART0_RXC_vect
  #define _DMX_USART_TXC_vect USART0_TXC_vect
  #define _DMX_USART_DRE_vect USART0_DRE_vect
  #define _DMX_USART_MUXMASK PORTMUX_USART0_gm
#elif (((defined(USARTNUM) && USARTNUM == 1) || !defined(USARTNUM)) && defined(USART1))
  #define _DMX_USART USART1
  #define _DMX_USART_RXC_vect USART1_RXC_vect
  #define _DMX_USART_TXC_vect USART1_TXC_vect
  #define _DMX_USART_DRE_vect USART1_DRE_vect
  #define _DMX_USART_MUXMASK PORTMUX_USART1_gm
#elif (((defined(USARTNUM) && USARTNUM == 2) || !defined(USARTNUM)) && defined(USART2))
  #define _DMX_USART USART2
  #define _DMX_USART_RXC_vect USART2_RXC_vect
  #define _DMX_USART_TXC_vect USART2_TXC_vect
  #define _DMX_USART_DRE_vect USART2_DRE_vect
  #define _DMX_USART_MUXMASK PORTMUX_USART2_gm
#elif (((defined(USARTNUM) && USARTNUM == 3) || !defined(USARTNUM)) && defined(USART3))
  #define _DMX_USART USART3
  #define _DMX_USART_RXC_vect USART3_RXC_vect
  #define _DMX_USART_TXC_vect USART3_TXC_vect
  #define _DMX_USART_DRE_vect USART3_DRE_vect
  #define _DMX_USART_MUXMASK PORTMUX_USART3_gm
#else
  #error "No USART specified in DMXSerial_megaavr - and neither 0 nor 1 exist!"
#endif



/// Initialize the Hardware MUX and UART serial port.
void _DMX_init()
{
  int32_t baud;
  #if defined(SIGROW_OSC16ERR5V) && (!defined(CLOCK_SOURCE) || CLOCK_SOURCE == 0)
  // Some modern AVR devices can run the internal oscillator at 16 OR 20 MHz and have factory calibrated error terms
    #if F_CPU == 16000000
      int8_t oscErr = SIGROW.OSC16ERR5V;
    #else 
      int8_t oscErr = SIGROW.OSC16ERR5V;
    #endif
    // calculate DMX speed Divider
    baud = (DMXSPEED * (1024 + oscErr)) / 1024;
    _DMX_dmxDivider = (64 * F_CPU) / (16 * baud);

    // calculate BREAK speed Divider
    baud = (BREAKSPEED * (1024 + oscErr)) / 1024;
    _DMX_breakDivider = (64 * F_CPU) / (16 * baud);
  #else
    // calculate DMX speed Divider
    baud = (DMXSPEED);
    _DMX_dmxDivider = (64 * F_CPU) / (16 * baud);

    // calculate BREAK speed Divider
    baud = (BREAKSPEED);
    _DMX_breakDivider = (64 * F_CPU) / (16 * baud);
  #endif
  // disable interrupts during initialization
  uint8_t oldSREG = SREG;
  cli();

  // Setup port mux
  #ifdef PORTMUX_USARTROUTEA
    PORTMUX.USARTROUTEA &= ~(_DMX_USART_MUXMASK);
    PORTMUX.USARTROUTEA |= _DMX_USARTMUXOPT;
  #else // tinyAVR 0/1-series, before they realized this was a dumb way to name them
    PORTMUX.CTRLB &= ~(_DMX_USART_MUXMASK);
    PORTMUX.CTRLB |= _DMX_USARTMUXOPT;
  #endif

  // Disable CLK2X, clock normal rate
  (_DMX_USART).CTRLB = USART_RXMODE_NORMAL_gc;

  //Set up the rx & tx pins
  pinMode(_DMX_USARTRX_PIN, INPUT_PULLUP);
  pinMode(_DMX_USARTTX_PIN, OUTPUT);

  // enable interrupts again, restore SREG content
  SREG = oldSREG;
} // _DMX_init()


/// Initialize the Hardware UART serial port to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
  uint16_t baud_setting;
  uint8_t flags;
  uint8_t format;

  // disable interrupts during initialization
  uint8_t oldSREG = SREG;
  cli();

  if (mode == DMXUARTMode::OFF) {
    // Disable transmitter and receiver
    (_DMX_USART).CTRLB = USART_RXMODE_NORMAL_gc; // (USART_RXEN_bm | USART_TXEN_bm);
    (_DMX_USART).CTRLA = 0; // disable all interrupts

  } else if (mode == DMXUARTMode::RONLY) {
    (_DMX_USART).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (_DMX_USART).CTRLC = DMXREADFORMAT; // accept data packets after first stop bit
    (_DMX_USART).CTRLB = USART_RXEN_bm | USART_RXMODE_NORMAL_gc; // Enable receiver only, normal speed
    (_DMX_USART).CTRLA = 0; // disable all interrupts

  } else if (mode == DMXUARTMode::RDATA) {
    (_DMX_USART).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (_DMX_USART).CTRLC = DMXREADFORMAT; // accept data packets after first stop bit
    (_DMX_USART).CTRLB = USART_RXEN_bm | USART_RXMODE_NORMAL_gc; // Enable receiver only, normal speed
    (_DMX_USART).CTRLA = USART_RXCIE_bm; // enable receive complete interrupt

  } else if (mode == DMXUARTMode::TBREAK) {
    // start UART with break settings, don't enable interrupts yet
    (_DMX_USART).CTRLB = 0; // no operation
    (_DMX_USART).CTRLA = 0; // disable all interrupts

    (_DMX_USART).BAUD = (int16_t)_DMX_breakDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (_DMX_USART).CTRLC = BREAKFORMAT; // Set USART mode of operation
    (_DMX_USART).CTRLB = USART_TXEN_bm; // Enable transmitter only, normal speed
    pinMode(_DMX_USARTTX_PIN, OUTPUT); // is required again after disabling UART
    (_DMX_USART).STATUS = USART_TXCIF_bm; //  clear transmit complete flag
    (_DMX_USART).CTRLA = USART_TXCIE_bm; // enable transmit complete interrupt

  } else if (mode == DMXUARTMode::TDATA) {
    // switch to dmx data mode
    (_DMX_USART).CTRLA = 0; // disable all interrupts
    (_DMX_USART).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (_DMX_USART).CTRLC = DMXFORMAT; // send with 2 stop bits for compatibility
    (_DMX_USART).CTRLA = USART_DREIE_bm; // enable data register empty interrupt

  } else if (mode == DMXUARTMode::TDONE) {
    (_DMX_USART).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (_DMX_USART).CTRLC = DMXFORMAT; // send with 2 stop bits for compatibility
    (_DMX_USART).STATUS = USART_TXCIF_bm; //  clear transmit complete flag
    (_DMX_USART).CTRLA = USART_TXCIE_bm; // enable transmit complete interrupt
  } // if

  // enable interrupts again, restore SREG content
  SREG = oldSREG;
} // _DMX_setMode()


// flush all incomming data packets in the queue
void _DMX_flush()
{
  uint8_t voiddata = (_DMX_USART).RXDATAL;
}


// send the next byte after current byte was sent completely.
inline void _DMX_writeByte(uint8_t data)
{
  // putting data into TXDATAL sends the data
  (_DMX_USART).TXDATAL = data;
} // _DMX_writeByte


// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte or frame error was received
ISR(_DMX_USART_RXC_vect)
{
  register8_t rxferr = (_DMX_USART).RXDATAH & USART_FERR_bm;
  register8_t rxdata = (_DMX_USART).RXDATAL;
  _DMXReceived(rxdata, rxferr);
} // ISR(USART1_RXC_vect)


// Interrupt service routines that are called when the actual byte was sent.
ISR(_DMX_USART_TXC_vect)
{
  _DMXTransmitted();
} // ISR(USART1_TXC_vect)


// this interrupt occurs after data register was emptied by handing it over to the shift register.
ISR(_DMX_USART_DRE_vect)
{
  _DMXTransmitted();
} // ISR(USART1_DRE_vect)

#endif

#endif
