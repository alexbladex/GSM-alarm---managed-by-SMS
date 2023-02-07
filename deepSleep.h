/*
 * deep sleep for 328p, 32u4
 *
 */
#ifndef deepSleep_h
#define deepSleep_h

#include <avr/sleep.h>
#include <avr/power.h>

//	Some macros is still missing from AVR GCC distribution for __AVR_ATmega32U4__
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
	// Timer 4 PRR bit is currently not defined in iom32u4.h
	#ifndef PRTIM4
		#define PRTIM4 4
	#endif

	// Timer 4 power reduction macro is not defined currently in power.h
	#ifndef power_timer4_disable
		#define power_timer4_disable()	(PRR1 |= (1u << PRTIM4))
	#endif

	#ifndef power_timer4_enable
		#define power_timer4_enable()	(PRR1 &= ~(1u << PRTIM4))
	#endif
#endif
void wakeupMode();

void deepSleep(byte bitReg = 0){
	bitReg = ~bitReg;
	//if bitReg = 0 these pins will always read as zero otherwise the corresponding pin can be enabled for interrupt
	//bit# - bitReg value
	//bit7
	//bit6
	//bit5
	//bit4
	//bit3 - 8 don't disable WDT
	//bit2 - 4 don't disable DIDR2
	//bit1 - 2 don't disable DIDR1
	//bit0 - 1 don't disable DIDR0
	asm volatile ("wdr");
	#ifdef USBCON
	uint8_t didr2 = DIDR2;
	USBDevice.detach();					// https://forum.arduino.cc/index.php?topic=186798#msg1384402
	if (bitReg & bit(2)) DIDR2 = 0xFF;	//ADC13..8 will always read as zero
	#else
	#endif

	uint8_t adcsra = ADCSRA, adcsrb = ADCSRB, acsr = ACSR, didr0 = DIDR0, didr1 = DIDR1;
	ADCSRA = 0;							//ADC is off
	ADCSRB = 0;							//AIN negative input
	ACSR  = 0;							//AIN positive input
	ACSR |= bit(ACD);					//Analog Comparator Disable
	if (bitReg & bit(1)) DIDR1 = 0xFF;	//AIN1/0 is disabled. The corresponding pin will always read as zero.
	if (bitReg & bit(0)) DIDR0 = 0xFF;	//ADC7..0 will always read as zero.
	if (bitReg & bit(3)) {
		WDTCSR |= bit(WDCE) | bit(WDE);	//Prepare to disable WDT
		WDTCSR  =  0;					//Disable WDT
	}
	power_all_disable();				//PRR = 0xFF; but first ADCSRA = 0
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	cli();							//Do not interrupt before we go to sleep
	MCUSR = 0;						//Clear various "reset" flags
	EIFR = 0xFF;					//Clear existing interrupt flags
	wakeupMode();					//attachInterrupt or WDT here
	sleep_enable();
	#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
	//It shut down BOD for 4 clocks only, so entering the sleep mode, should be next instruction.
	//Upon wake-up from sleep, BOD is automatically enabled again.
	sleep_bod_disable();			//Disable Brown Out Detector if available. Will add 60uS to wake up time but save power in sleep mode
	#endif
	sei();
	sleep_cpu();					//Sleep here and wake up here if triggered

	sleep_disable();
	power_all_enable();
	#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
	//delay(5);
	#endif
	ADCSRA = adcsra;
	ADCSRB = adcsrb;
	DIDR0 = didr0;
	DIDR1 = didr1;
	ACSR = acsr;

	#ifdef USBCON
	DIDR2 = didr2;
	USBDevice.attach();
	#else
	#endif
}

#endif //deepSleep_h