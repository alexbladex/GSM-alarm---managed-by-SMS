/*
 * nanoWDT methods for 328p, 32u4 with added interrupt reset mode
 *
 */
#ifndef _AVR_WDT_H_
#define _AVR_WDT_H_
#ifndef WiringPrivate_h
static void nothing(){}
typedef void (*voidFuncPtr)();
#endif

#define WDTO_15MS    0
#define WDTO_30MS                           (1 << WDP0)
#define WDTO_60MS               (1 << WDP1)
#define WDTO_120MS (            (1 << WDP1)|(1 << WDP0))
#define WDTO_250MS  (1 << WDP2)
#define WDTO_500MS ((1 << WDP2)|            (1 << WDP0))
#define WDTO_1S    ((1 << WDP2)|(1 << WDP1))
#define WDTO_2S    ((1 << WDP2)|(1 << WDP1)|(1 << WDP0))
 #ifdef WDP3
#define WDTO_4S     (1 << WDP3)
#define WDTO_8S    ((1 << WDP3)|            (1 << WDP0))
 #endif

#define WDT_RESET            (1 << WDE)
#define WDT_INTERRUPT        (1 << WDIE)
#define WDT_INTERRUPT_RESET ((1 << WDIE)|(1 << WDE))

static volatile voidFuncPtr wdtFunc;

inline void wdt_reset() {asm volatile ("wdr");}

inline void wdt_enable(uint8_t timeout = WDTO_1S, uint8_t mode = WDT_RESET, void (*userFunc)() = nothing) {
  cli();
  asm volatile ("wdr");
  WDTCSR |= bit(WDCE) | bit(WDE);
  WDTCSR  = mode | timeout;
  wdtFunc = userFunc;
  sei();
}

inline void wdt_disable() {
  asm volatile ("wdr");
  uint8_t sreg = SREG;
  cli();
  MCUSR &= ~bit(WDRF);
  WDTCSR |= bit(WDCE) | bit(WDE);
  WDTCSR  =  0;
  wdtFunc = nothing;
  SREG = sreg; //In case if Interrupt was disabled before wdt_disable();
}

#ifdef WDT_vect
  ISR(WDT_vect)
#else
  #error "Don't know what the WDT vector is called"
#endif
  {
    #ifdef USE_WDT_RESET
    asm volatile ("wdr");
    #endif
    wdtFunc();
  }

#endif //_AVR_WDT_H_