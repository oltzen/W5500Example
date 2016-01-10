/*
 * tom_io_common.h
 *
 * Created: 04.01.2016 15:35:28
 *  Author: Thomas
 */ 
//! Copyright (c)  2016 Thomas Oltzen
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.

#ifndef TOM_IO_COMMON_H_
#define TOM_IO_COMMON_H_
#include <avr/io.h>

#define	PIN_B0_ON PORTB |= (1<<0)
#define	PIN_B1_ON PORTB |= (1<<1)
#define	PIN_B2_ON PORTB |= (1<<2)
#define	PIN_B3_ON PORTB |= (1<<3)
#define	PIN_B4_ON PORTB |= (1<<4)
#define	PIN_B5_ON PORTB |= (1<<5)
#define	PIN_B6_ON PORTB |= (1<<6)
#define	PIN_B7_ON PORTB |= (1<<7)

#define PIN_B0_OFF PORTB &= (~(1<<0))
#define PIN_B1_OFF PORTB &= (~(1<<1))
#define PIN_B2_OFF PORTB &= (~(1<<2))
#define PIN_B3_OFF PORTB &= (~(1<<3))
#define PIN_B4_OFF PORTB &= (~(1<<4))
#define PIN_B5_OFF PORTB &= (~(1<<5))
#define PIN_B6_OFF PORTB &= (~(1<<6))
#define PIN_B7_OFF PORTB &= (~(1<<7))

#define	PIN_C0_ON PORTC |= (1<<0)
#define	PIN_C1_ON PORTC |= (1<<1)
#define	PIN_C2_ON PORTC |= (1<<2)
#define	PIN_C3_ON PORTC |= (1<<3)
#define	PIN_C4_ON PORTC |= (1<<4)
#define	PIN_C5_ON PORTC |= (1<<5)
#define	PIN_C6_ON PORTC |= (1<<6)
#define	PIN_C7_ON PORTC |= (1<<7)

#define PIN_C0_OFF PORTC &= (~(1<<0))
#define PIN_C1_OFF PORTC &= (~(1<<1))
#define PIN_C2_OFF PORTC &= (~(1<<2))
#define PIN_C3_OFF PORTC &= (~(1<<3))
#define PIN_C4_OFF PORTC &= (~(1<<4))
#define PIN_C5_OFF PORTC &= (~(1<<5))
#define PIN_C6_OFF PORTC &= (~(1<<6))
#define PIN_C7_OFF PORTC &= (~(1<<7))

#define	PIN_D0_ON PORTD |= (1<<0)
#define	PIN_D1_ON PORTD |= (1<<1)
#define	PIN_D2_ON PORTD |= (1<<2)
#define	PIN_D3_ON PORTD |= (1<<3)
#define	PIN_D4_ON PORTD |= (1<<4)
#define	PIN_D5_ON PORTD |= (1<<5)
#define	PIN_D6_ON PORTD |= (1<<6)
#define	PIN_D7_ON PORTD |= (1<<7)

#define PIN_D0_OFF PORTD &= (~(1<<0))
#define PIN_D1_OFF PORTD &= (~(1<<1))
#define PIN_D2_OFF PORTD &= (~(1<<2))
#define PIN_D3_OFF PORTD &= (~(1<<3))
#define PIN_D4_OFF PORTD &= (~(1<<4))
#define PIN_D5_OFF PORTD &= (~(1<<5))
#define PIN_D6_OFF PORTD &= (~(1<<6))
#define PIN_D7_OFF PORTD &= (~(1<<7))

// Data direction of pins

#define PORTB_ALL_OUT DDRB = 0xff
#define PORTB_ALL_IN DDRB = 0
#define PORTC_ALL_OUT DDRC = 0xff
#define PORTC_ALL_IN DDRC = 0
#define PORTD_ALL_OUT DDRD = 0xff
#define PORTD_ALL_IN DDRD = 0

#define	PIN_B0_OUT DDRB |= (1<<0)
#define	PIN_B1_OUT DDRB |= (1<<1)
#define	PIN_B2_OUT DDRB |= (1<<2)
#define	PIN_B3_OUT DDRB |= (1<<3)
#define	PIN_B4_OUT DDRB |= (1<<4)
#define	PIN_B5_OUT DDRB |= (1<<5)
#define	PIN_B6_OUT DDRB |= (1<<6)
#define	PIN_B7_OUT DDRB |= (1<<7)

#define PIN_B0_IN DDRB &= (~(1<<0))
#define PIN_B1_IN DDRB &= (~(1<<1))
#define PIN_B2_IN DDRB &= (~(1<<2))
#define PIN_B3_IN DDRB &= (~(1<<3))
#define PIN_B4_IN DDRB &= (~(1<<4))
#define PIN_B5_IN DDRB &= (~(1<<5))
#define PIN_B6_IN DDRB &= (~(1<<6))
#define PIN_B7_IN DDRB &= (~(1<<7))

#define	PIN_C0_OUT DDRC |= (1<<0)
#define	PIN_C1_OUT DDRC |= (1<<1)
#define	PIN_C2_OUT DDRC |= (1<<2)
#define	PIN_C3_OUT DDRC |= (1<<3)
#define	PIN_C4_OUT DDRC |= (1<<4)
#define	PIN_C5_OUT DDRC |= (1<<5)
#define	PIN_C6_OUT DDRC |= (1<<6)
#define	PIN_C7_OUT DDRC |= (1<<7)

#define PIN_C0_IN DDRC &= (~(1<<0))
#define PIN_C1_IN DDRC &= (~(1<<1))
#define PIN_C2_IN DDRC &= (~(1<<2))
#define PIN_C3_IN DDRC &= (~(1<<3))
#define PIN_C4_IN DDRC &= (~(1<<4))
#define PIN_C5_IN DDRC &= (~(1<<5))
#define PIN_C6_IN DDRC &= (~(1<<6))
#define PIN_C7_IN DDRC &= (~(1<<7))

#define	PIN_D0_OUT DDRD |= (1<<0)
#define	PIN_D1_OUT DDRD |= (1<<1)
#define	PIN_D2_OUT DDRD |= (1<<2)
#define	PIN_D3_OUT DDRD |= (1<<3)
#define	PIN_D4_OUT DDRD |= (1<<4)
#define	PIN_D5_OUT DDRD |= (1<<5)
#define	PIN_D6_OUT DDRD |= (1<<6)
#define	PIN_D7_OUT DDRD |= (1<<7)

#define PIN_D0_IN DDRD &= (~(1<<0))
#define PIN_D1_IN DDRD &= (~(1<<1))
#define PIN_D2_IN DDRD &= (~(1<<2))
#define PIN_D3_IN DDRD &= (~(1<<3))
#define PIN_D4_IN DDRD &= (~(1<<4))
#define PIN_D5_IN DDRD &= (~(1<<5))
#define PIN_D6_IN DDRD &= (~(1<<6))
#define PIN_D7_IN DDRD &= (~(1<<7))

#endif /* TOM_IO_COMMON_H_ */