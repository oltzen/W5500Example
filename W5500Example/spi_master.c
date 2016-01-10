/*
 * spi_master.c
 *
 * Created: 08.01.2016 08:27:52
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
#include "tom_atmega8_io.h"

void SPI_MasterInit(void)
{
	
	//DDRB = ((1<<DDB2)|(1<<DDB1)|(1<<DDB0)); //spi pins on port b MOSI SCK,SS outputs
	PIN_B0_OUT; // SS Line
	PIN_B3_OUT; // MOSI
	PIN_B5_OUT; // SCK
	PIN_B2_OUT;// SS Line to W5500
	PIN_B2_ON;
	PIN_B0_ON;
	SPCR |= ((1<<SPE)|(1<<MSTR)|(1<<SPR0));  // SPI enable, Master, f/16
	
}

uint8_t SPI_TransferByte (uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF))){
	}
	return SPDR;
}
