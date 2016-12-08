/*************************************************************************
Title:    MRBus MRB-BD42 v3 firmware
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2015 Nathan Holmes

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
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbus.h"

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ 0x01

uint8_t internalBDStatus = 0;
uint8_t externalBDStatus = 0;

uint8_t channelOnDelayCount[4] = {0, 0, 0, 0};
uint8_t channelOffDelayCount[4] = {0, 0, 0, 0};
uint8_t channelOnDelay[4] = {4, 4, 4, 4};
uint8_t channelOffDelay[4] = {25, 25, 25, 25};

uint16_t eepromThreshold[4];

#define MRBUS_EE_CHANNEL0_ON_DELAY  0x10
#define MRBUS_EE_CHANNEL1_ON_DELAY  0x11
#define MRBUS_EE_CHANNEL2_ON_DELAY  0x12
#define MRBUS_EE_CHANNEL3_ON_DELAY  0x13

#define MRBUS_EE_CHANNEL0_OFF_DELAY  0x20
#define MRBUS_EE_CHANNEL1_OFF_DELAY  0x21
#define MRBUS_EE_CHANNEL2_OFF_DELAY  0x22
#define MRBUS_EE_CHANNEL3_OFF_DELAY  0x23

#define MRBUS_EE_CHANNEL0_THRESHOLD_H 0x30
#define MRBUS_EE_CHANNEL0_THRESHOLD_L 0x31

volatile uint16_t adcValue[8];


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks = 0;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;
volatile uint8_t busTimeout = 0;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}


ISR(TIMER0_COMPA_vect)
{
	uint8_t newLedValue;
	
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}

	if (busTimeout)
		busTimeout--;

	if (ticks & 0x01)
	{
		newLedValue = 0xE0;
	
		if (internalBDStatus & 0x01)
			newLedValue &= ~0x20;
		if (internalBDStatus & 0x02)
			newLedValue &= ~0x40;
	} else {
		newLedValue = 0x00;

		if (internalBDStatus & 0x04)
			newLedValue |= 0x20;
		if (internalBDStatus & 0x08)
			newLedValue |= 0x40;
	}

	PORTD = (PORTD & 0x1F) | newLedValue;
}

// End of 100Hz timer

void initializeADC()
{
	for(uint8_t i=0; i<8; i++)
		adcValue[i] = 0;

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x40;  // AVCC reference, ADC0 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x3F;  // Turn all ADC pins 0-5 into analog inputs
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

ISR(ADC_vect)
{
	static uint8_t workingChannel = 0;
	static uint16_t accumulator = 0;
	static uint8_t count = 0;
	
	accumulator += ADC;
	if (++count >= 64)
	{
		adcValue[workingChannel] = accumulator / 64;
		accumulator = 0;
		count = 0;
		workingChannel++;
		
		ADMUX = (ADMUX & 0xF0) | (workingChannel & 0x07);
		
		if (8 == workingChannel)
		{
			workingChannel = 0;
			eventFlags |= EVENT_DO_BD_READ;
		}
	}

	if (0 == (eventFlags & EVENT_DO_BD_READ))
	{
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
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
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRED;
#ifndef GIT_REV
// This stubs in GIT_REV to 0 in case the make system isn't providing the current SVN revisions
#define GIT_REV=0L
#endif				
		txBuffer[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
		txBuffer[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
		txBuffer[9]  = 0xFF & (GIT_REV); // Software Revision
		txBuffer[10]  = 3; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'B';
		txBuffer[13] = 'D';
		txBuffer[14] = '4';
		txBuffer[15] = '2';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	
	
	return;	
}

#define PORTB_IN_BITS 0x3F
#define PORTC_IN_BITS 0xFF
#define PORTD_IN_BITS 0x18

void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	// Turn the correct bits to inputs
	DDRB = ~(PORTB_IN_BITS);
	DDRC = 0xFF & ~(PORTC_IN_BITS);
	DDRD = ~(PORTD_IN_BITS);
	
	// Turn on pull-ups
	PORTB |= PORTB_IN_BITS;
	PORTC |= PORTC_IN_BITS;
	PORTD |= PORTD_IN_BITS;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	if (0xFFFF == update_decisecs)
	{
		// It's uninitialized - go ahead and set it to 2 seconds
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, 20);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0);
	}
	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);

	for (uint8_t channel = 0; channel < 4; channel++)
	{
		channelOnDelay[channel] = eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_ON_DELAY + channel));
		if (0xFF == channelOnDelay[channel] || 0x00 == channelOnDelay[channel])
		{
			eeprom_write_byte((uint8_t*)(MRBUS_EE_CHANNEL0_ON_DELAY + channel), 4);
			channelOnDelay[channel] = eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_ON_DELAY + channel));			
		}

		channelOffDelay[channel] = eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_OFF_DELAY + channel));
		if (0xFF == channelOffDelay[channel] || 0x00 == channelOnDelay[channel])
		{
			eeprom_write_byte((uint8_t*)(MRBUS_EE_CHANNEL0_OFF_DELAY + channel), 25);
			channelOffDelay[channel] = eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_OFF_DELAY + channel));			
		}
		
		eepromThreshold[channel] = (uint16_t)eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_L + channel*2)) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_H + channel * 2))) << 8);
		if (eepromThreshold[channel] > 1023)
		{
			eeprom_write_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_L + channel*2), 0xFF);
			eeprom_write_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_H + channel*2), 0xFF);
			eepromThreshold[channel] = (uint16_t)eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_L + channel*2)) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)(MRBUS_EE_CHANNEL0_THRESHOLD_H + channel * 2))) << 8);
		}
		
	}
}


void readInternalDetectors(void)
{	
	for(uint8_t channel = 0; channel < 4; channel++)
	{
		uint16_t threshold = (0xFFFF == eepromThreshold[channel])?adcValue[channel+4]:eepromThreshold[channel];
		uint8_t channelMask = (1<<channel);

		// Introduce a bit of hysteresis. 
		// If the channel is currently "on", lower the threshold by 5 counts
		// If the channel is currently "off", raise the threshold by 5
		if ((internalBDStatus & channelMask) && (threshold >= 5))
		{
			threshold -= 5;
		} 
		else if ( (0 == (internalBDStatus & channelMask)) && (threshold <= (1023-5)) )
		{
			threshold += 5;
		}
	
		if (adcValue[channel] > threshold)
		{
			// Current for the channel has exceeded threshold
			
			if (0 == (internalBDStatus & channelMask))
			{
				// Channel is currently in a non-detecting state
				// Wait for the turnon delay before actually indicating on
				if (channelOnDelayCount[channel] < channelOnDelay[channel])
					channelOnDelayCount[channel]++;
				else
				{
					internalBDStatus |= channelMask;
					channelOffDelayCount[channel] = 0;
				}
			} else {
				// Channel is currently in a detecting state
				channelOffDelayCount[channel] = 0;
			}
		} else {
			// Current for the channel is under the threshold value
			if (internalBDStatus & channelMask)
			{
				// Channel is currently in a non-detecting state
				if (channelOffDelayCount[channel] < channelOffDelay[channel])
					channelOffDelayCount[channel]++;
				else
				{
					internalBDStatus &= ~(channelMask);
					channelOnDelayCount[channel] = 0;
				}
			} else {
				// Channel is currently in a detecting state
				channelOnDelayCount[channel] = 0;
			}
		}
	}
}

uint8_t debounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = newInputs ^ debouncedState;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}

void readExternalDetectors()
{
	uint8_t currentInputs = (PINB & 0x3F) | (0xC0 & (PIND<<3));
	externalBDStatus = debounce(externalBDStatus, currentInputs);
}

uint8_t readDetectors(void)
{
	static uint8_t old_internalBDStatus = 0;
	static uint8_t old_externalBDStatus = 0;
	uint8_t changed = 0;
	
	// Okay, we've got a new scan of all ADC inputs
	// Do some stuff
	readInternalDetectors();
	readExternalDetectors();

	if (old_internalBDStatus != internalBDStatus
		|| old_externalBDStatus != externalBDStatus)
	{
		old_internalBDStatus = internalBDStatus;
		old_externalBDStatus = externalBDStatus;
		changed = 1;
	}
	
	// Returns 0 if no change, 1 if change
	return changed;
}


int main(void)
{
	uint8_t changed = 0;

	// Application initialization
	init();
	initializeADC();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();

		if (eventFlags & EVENT_DO_BD_READ)
		{
			changed = readDetectors();
			
			eventFlags &= ~(EVENT_DO_BD_READ);
			ADCSRA |= _BV(ADSC);			
		}

		if (decisecs >= update_decisecs)		
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				decisecs -= update_decisecs;
			}
			changed = 1;
		}

		// If we need to send a packet and the TX queue isn't full...
		if (changed && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[10];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[5] = 'S';
			txBuffer[6] = internalBDStatus;
			txBuffer[7] = externalBDStatus;

			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			changed = 0;
		}

		if (mrbusPktQueueDepth(&mrbusTxQueue) && 0 == busTimeout)
		{
			uint8_t fail = mrbusTransmit();
			// If we failed to transmit, just take some number of 10mS increments to wait until trying again, minimum of 40mS
			if (fail)
				busTimeout = 4 + (mrbus_dev_addr & 0x0F);
		}
	}
}



