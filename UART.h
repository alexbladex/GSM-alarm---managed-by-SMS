/*
 * nanoUART class main goal is to reduce size and increase speed for 328p, 32u4
 *
 */

#ifndef UART_h
#define UART_h
#include "nStream.h"

//  http://we.easyelectronics.ru/AVR/taymery-i-zaderzhki-sbornik-receptov.html
//  Serial Interface for ATMega 168/328
//  Asynchronous Mode with Normal or Double speed
//  Example: UART<data_type, buffer_size> serial(baud_rate);
//  data_type: uint8_t by default
//  buffer_size: must be a power of 2 & max 256
//  baud: 9600 (default), 19200, 31250 (MIDI) 38400 57600 115200 baud rate
//  frame format: 8N1 (default)
//  u2x: 1 (default double), 0 (normal) transfer rate, if many errors set to 0
//  https://github.com/SlashDevin/NeoSWSerial
//  Define frame format for serial.begin(baud, format);
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

//#include <assert.h>
#define rx_buffer_size 16
#define rx_mask (rx_buffer_size - 1)
//Enable Receiver, Transmitter and Interrupt here:
#define rxen  1
#define txen  1
#define rxcie 1
#define txcie 0
#define udrie 0

class UART : public nStream {
    // inline member function should be defined in header
  private:
    typedef uint8_t B;
    //static const B rx_buffer_size = 16;
    //static const B rx_mask = rx_buffer_size - 1;
    volatile B rx_head = 0;
    volatile B rx_tail = 0;
    B rx_buffer[rx_buffer_size] = {0};
    static_assert(rx_buffer_size >= 2, "buffer size must be more than 0");
    static_assert(rx_buffer_size <= 256, "buffer size must be less than 256");
    static_assert((rx_buffer_size & rx_mask) == 0, "buffer size must be a power of 2");
    //const B rxen, txen, rxcie, txcie, udrie;

  public:
    /*
    UART(int rxen_= 1, int txen_= 1, int rxcie_= 1, int txcie_= 0, int udrie_= 0)
    : rxen(rxen_), txen(txen_), rxcie(rxcie_), txcie(txcie_), udrie(udrie_)
    {
      assert(rxen_ == 0 || rxen_ == 1);
      assert(txen_ == 0 || txen_ == 1);
      assert(rxcie_== 0 || rxcie_== 1);
      assert(txcie_== 0 || txcie_== 1);
      assert(udrie_== 0 || udrie_== 1);
    }
    */
    inline void begin(const uint32_t baud = 9600, const uint8_t format = 6, const bool u2x = 1) {
      /* USART Initialization */
      // Enable pull-up on RX pin, in order to suppress line noise
      #ifdef USBCON
      DDRD &= ~bit(PD2);
      PORTD |= bit(PD2);
      #else
      DDRD &= ~bit(PD0);
      PORTD |= bit(PD0);
      #endif
      cli();
      // Set baud rate
      if (u2x) {
        UCSR0A = bit(U2X0);
        UBRR0H = (B)((F_CPU / 8UL / baud - 1) >> 8);
        UBRR0L = (B)(F_CPU / 8UL / baud - 1);
      }
      else {
        UCSR0A = 0;
        UBRR0H = (B)((F_CPU / 16UL / baud - 1) >> 8);
        UBRR0L = (B)(F_CPU / 16UL / baud - 1);
      }
      // Enable receiver, transmitter, interrupt
      UCSR0B = (rxen << RXEN0) | (txen << TXEN0) | (rxcie << RXCIE0) | (txcie << TXCIE0) | (udrie << UDRIE0);
      // Set frame format 8N1: (3 << UCSZ00)
      UCSR0C = format;
      sei();
    }

    inline void end() {
      uint8_t sreg = SREG;
      cli();
      UCSR0B = 0;
      rx_tail = rx_head;
      SREG = sreg; // In case if Interrupt was disabled before disable USART
    }

    // Receive Complete Interrupt
    // ISR(USART_RX_vect) {
    //   bool udr = UART::receive();
    //   #ifdef USE_WDT_RESET
    //   if (udr) asm volatile ("wdr");
    //   #endif
    // }
    inline bool receive() {
      // Wait while UDR buffer is empty. RXC0 = 1 has unread data
      while (!(UCSR0A & (1 << RXC0)));
      // If Parity error, read and discard byte
      if (UCSR0A & ((1 << FE0)|(1 << DOR0)|(1 << UPE0))) { UDR0; return false; }
      // If buffer is not full, read and write to rx_buffer
      if ((B)(rx_head - rx_tail) < rx_buffer_size) rx_buffer[rx_head++ & rx_mask] = UDR0;
      return true;
    }

    using nPrint::write; // pull in write(const char *);
    inline void write(uint32_t c){ write((B)c); }
    inline void write(uint16_t c){ write((B)c); }
    inline void write(int32_t c) { write((B)c); }
    inline void write(int16_t c) { write((B)c); }
    inline void write(const B c) {
      // Clear transmit flag
      UCSR0A |= bit(TXC0);
      // Wait until buffer is empty. UDRE0 = 1 can receive new data. TXC0 = 1 transmit completed
      while (!(UCSR0A & (1 << UDRE0)));
      // Put data into buffer
      UDR0 = c;
      // while (!(UCSR0A & (1 << TXC0))); /* If many error comment this */
      // return (UCSR0A & (1 << UDRE0));
    }

    using nStream::read; // pull in read(char *, int);
    inline B read() {
      return rx_tail == rx_head ? 0 : rx_buffer[rx_tail++ & rx_mask];
    }

    inline B peek() {
      return rx_tail == rx_head ? 0 : rx_buffer[rx_tail & rx_mask];
    }

    // Flushing MCU receive buffer
    inline void flush(){
      while (UCSR0A & (1 << RXC0)) UDR0;
    }

    // Clearing buffer
    inline void clear() {
      rx_tail = rx_head;
    }

    inline B count() const {
      return rx_head - rx_tail;
    }

    inline B size() const {
      return rx_buffer_size;
    }

    inline bool available() const {
      return count();
    }

};

#ifdef USBCON
  UART serial1;
  #define HAVE_HWSERIAL1
#else
  UART serial;
  #define HAVE_HWSERIAL0
#endif

#if defined(USART_RX_vect)
  ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
  ISR(USART0_RX_vect)
#elif defined(USART_RXC_vect)
  ISR(USART_RXC_vect) // ATmega8
#elif defined(USART1_RX_vect)
  ISR(USART1_RX_vect) // ATmega32u4
#elif defined(UART1_RX_vect)
  ISR(UART1_RX_vect) // ATmega32u4
#else
  #error "Don't know what the Receiver vector is called for Serial"
#endif
  {
    #ifdef USBCON
    //bool udr = serial1.receive();
	serial1.receive();
    #else
    //bool udr = serial.receive();
    serial.receive();
	#endif
    //if (udr) asm volatile ("wdr");
  }
#endif //UART_h