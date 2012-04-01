/*************************************************************************
Title:    MRBus 4-Channel Block Detector, Mark II
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "mrbus.h"

extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;
extern uint8_t mrbus_activity;

uint8_t mrbus_dev_addr = 0x11;

// ******** Start 100 Hz Timer 

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint8_t ticks;
uint8_t secs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	secs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 100)
	{
		ticks = 0;
		secs++;
	}
	PORTB ^= _BV(PB0);
}
/*
void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	secs = 0;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;
	ticks++;
	if (ticks >= 100)
	{
		ticks = 0;
		secs++;
	}
}

// ******** End 100 Hz Timer 
*/
#define UINT16_HIGH_BYTE(a)  ((a)>>8)
#define UINT16_LOW_BYTE(a)  ((a) & 0xFF)

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

void init(void)
{
	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
}


int main(void)
{
	uint8_t changed=0;
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	DDRB |= _BV(PB0);
	
	// Initialize MRBus core
	mrbusInit();

	sei();	

	while (1)
	{
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();
			
		// FIXME: Do any module-specific behaviours here in the loop.

		/* Events that happen every second */
		if (secs >= 2)
		{
			secs = 0;
			changed = 1;		
		}

		/* If we need to send a packet and we're not already busy... */
		if ((changed != 0) && !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = 0;
			mrbus_state |= MRBUS_TX_PKT_READY;
			changed = 0;
		}
		

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
		}
	}
}



