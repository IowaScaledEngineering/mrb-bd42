/*************************************************************************
Title:    MRB-BD4X v3.1 firmware
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2016 Nathan Holmes and Michael Petersen

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

volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ 0x01

uint8_t internalBDStatus = 0;
uint8_t externalBDStatus = 0;

uint8_t channelOnDelayCount[4] = {0, 0, 0, 0};
uint8_t channelOffDelayCount[4] = {0, 0, 0, 0};
uint8_t channelOnDelay[4] = {4, 4, 4, 4};
uint8_t channelOffDelay[4] = {25, 25, 25, 25};

uint16_t eepromThreshold[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

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

void setPktLED(uint8_t on)
{
	if (on)
		PORTD &= ~(0x01);
	else
		PORTD |= 0x01;
}

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x4E;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t ticks = 0;
	uint8_t newLedValue;
	
	if (++ticks >= 100)
		ticks = 0;

	// Blink the packet LED at 1Hz to indicate life
	setPktLED((ticks>50)?1:0);

	// Multiplex the block detector indicator lights
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
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler (125kHz ADC clock on 8MHz internal clock)
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
			// EVENT_DO_BD_READ is an interlock - once we've completed all eight channels, set it
			// so that the main loop can then process what we've collected
			eventFlags |= EVENT_DO_BD_READ;  
		}
	}


	// If we're not done reading (ie, the flag is set), go ahead and trigger the next conversion
	// Not using auto-trigger so that we can safely change channels - avoids that first glitch
	// reading
	if (0 == (eventFlags & EVENT_DO_BD_READ))
	{
		ADCSRA |= _BV(ADSC);
	}
}

#define PORTB_IN_BITS 0x30
#define PORTC_IN_BITS 0xFF
#define PORTD_IN_BITS 0x18

void init(void)
{
	// Clear watchdog
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
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;


	// Turn the correct bits to inputs
	DDRB = ~(PORTB_IN_BITS);
	DDRC = 0xFF & ~(PORTC_IN_BITS);
	DDRD = ~(PORTD_IN_BITS);
	
	// Turn on pull-ups
	PORTB |= PORTB_IN_BITS;
	PORTC |= PORTC_IN_BITS;
	PORTD |= PORTD_IN_BITS;

	setPktLED(0);

	// Read all four channel configurations from EEPROM (on delay, off delay, firmware thresholds)
	// If they aren't initialized (set to 0xFF), initialize them and reread

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

int main(void)
{
	// Application initialization
	init();
	initializeADC();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	sei();	

	while (1)
	{
		wdt_reset();

		if (eventFlags & EVENT_DO_BD_READ)
		{
			readInternalDetectors();
			PORTB = (PORTB & 0xF0) | (internalBDStatus & 0x0F);
			
			eventFlags &= ~(EVENT_DO_BD_READ);
			ADCSRA |= _BV(ADSC);  // Start next ADC round
		}
	}
}



