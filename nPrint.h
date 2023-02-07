/*
 * nanoPrint class w/o String object support
 * based on tttapa and DrOffset recomondations
 *
 */
#ifndef nPrint_h
#define nPrint_h
#include <Arduino.h>
#ifdef BIN // prevent warnings if BIN is previously defined in "iotnx4.h" or similar
#undef BIN
#endif
#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2
enum _endline { endl };
enum _baseint { dec, hex, bin, oct };
#ifndef F
class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#endif

class nPrint {

  public:
    virtual void write(unsigned char) = 0;

    inline unsigned char getBase() {
      if (output.base == 0) return DEC;
      if (output.base == 1) return HEX;
      if (output.base == 2) return BIN;
      return OCT;
    }

    inline unsigned char getPrecision() {
      if (output.precision == 0) return 2;
      if (output.precision == 1) return 3;
      if (output.precision == 2) return 4;
      return 6;
    }

    inline void setPrecision(unsigned char n) {
      // 0(.2), 1(.3), 2(.4), 3(.6)
      if      (n <= 2) output.precision = 0;
      else if (n == 3) output.precision = 1;
      else if (n == 4) output.precision = 2;
      else             output.precision = 3;
    }

    inline void setBase(unsigned char n) {
      // 0 DEC, 1 HEX, 2 BIN, 3 OCT
      if      (n == HEX) output.base = 1;
      else if (n == BIN) output.base = 2;
      else if (n == DEC) output.base = 0;
      else               output.base = 3;
    }

    inline void setOutputbase(unsigned char n) {
      // 0 DEC, 1 HEX, 2 BIN, 3 OCT
      output.base = n;
    }

    inline void setLeadingZero(bool lead_zero) {
      output.leadzero = lead_zero;
    }

    inline void setUppercase(bool upper_case) {
      output.uppercase = upper_case;
    }

    inline void setBoolalpha(bool bool_alpha) {
      output.boolalpha = bool_alpha;
    }

    inline void setShowbase(bool show_base) {
      output.showbase = show_base;
    }

    inline void write(const char *s) {
      while (*s) write(*s++);
    }

    inline void write_P(const char *i) {
      while (pgm_read_byte(i)) write(pgm_read_byte(i++));
    }

    inline void print_P(const char *i) {
      while (pgm_read_byte(i)) write(pgm_read_byte(i++));
    }

    inline void println_P(const char *i) {
      print_P(i);
      println();
    }

    inline void print(const __FlashStringHelper *i) {
      print_P((char*)i);
    }

    inline void println(const __FlashStringHelper *i) {
      print_P((char*)i);
      println();
    }

    inline void print(char c) {
      write(c);
    }

    inline void print(const char *s) {
      while (*s) write(*s++);
    }

    inline void print(unsigned char b, int base = DEC) {
      print((unsigned long) b, base);
    }

    inline void print(int n, int base = DEC) {
      print((long) n, base);
    }

    inline void print(unsigned int n, int base = DEC) {
      print((unsigned long) n, base);
    }

    inline void print(long n, int base = DEC) {
      if (base == DEC) {
        if (n < 0) {
          print('-');
          n = -n;
        }
        printNumber(n, DEC);
      }
      else {
        printNumber(n, base);
      }
    }

    inline void print(unsigned long n, int base = DEC) {
      printNumber(n, base);
    }

    inline void print(double n, int digits = 2) {
      printFloat(n, digits);
    }

    inline void print(bool b) {
      if (output.boolalpha) print_P(b ? PSTR("true") : PSTR("false"));
      else print((int)b);
    }

    inline void println() {
      print('\r'); print('\n');
    }

    inline void println(char c) {
      print(c);
      println();
    }

    inline void println(const char *s) {
      print(s);
      println();
    }

    inline void println(unsigned char b, int base = DEC) {
      print(b, base);
      println();
    }

    inline void println(int num, int base = DEC) {
      print(num, base);
      println();
    }

    inline void println(unsigned int num, int base = DEC) {
      print(num, base);
      println();
    }

    inline void println(long num, int base = DEC) {
      print(num, base);
      println();
    }

    inline void println(unsigned long num, int base = DEC) {
      print(num, base);
      println();
    }

    inline void println(double num, int digits = 2) {
      print(num, digits);
      println();
    }

    inline void println(bool b) {
      print(b);
      println();
    }

    inline nPrint& operator<<(const int& i) {
      print(i, getBase());
      return *this;
    }

    inline nPrint& operator<<(const long& i) {
      print(i, getBase());
      return *this;
    }

    inline nPrint& operator<<(const unsigned char& i) {
      print(i, getBase());
      return *this;
    }

    inline nPrint& operator<<(const unsigned int& i) {
      print(i, getBase());
      return *this;
    }

    inline nPrint& operator<<(const unsigned long& i) {
      print(i, getBase());
      return *this;
    }

    inline nPrint& operator<<(const double& i) {
      print(i, getPrecision());
      return *this;
    }

    template <typename T>
    inline nPrint& operator<<(const T& i) {
      print(i);
      return *this;
    }

    template <typename T>
    inline T& operator<<(T& (*manipulator)(T&)) {
      return manipulator(*this);
    }

  protected:
    struct stream {
      unsigned char base : 2; // 0 DEC, 1 HEX, 2 BIN, 3 OCT
      unsigned char precision : 2; // 0(.2), 1(.3), 2(.4), 3(.6)
      bool leadzero : 1, uppercase : 1, boolalpha : 1, showbase : 1;
    } output = {0, 0, 1, 1, 1, 1};

    void printNumber(unsigned long n, int base) {
      //const unsigned char alpha = uppercase ? 55 : 87;
      char buf[8 * sizeof(long) + 3]; //for BIN each byte * 8 bits + null terminator, base and sign
      char *str = &buf[sizeof(buf) - 1];
      unsigned long bits = n;
      unsigned char c;
      *str = '\0';

      /* prevent crash if called with base == 1 */
      if (base < 2) base = DEC;
      do {
        //*--str = "0123456789ABCDEF"[n % base];
        c = n % base;
        n /= base;
        *--str = c < 10 ? c + '0' : c + 55; // +55 A..F
      } while (n);

      if (base == DEC) {
      } else
      if (base == BIN) {
        if (output.leadzero) {
          if (bits > 0xFFFF) {
            if (bits > 0xFFFFFF) bits = 32;
            else                 bits = 24;
          } else {
            if (bits > 0xFF)     bits = 16;
            else                 bits = 8;
          }
          bits = bits - strlen(str);
          while (bits--) *--str = '0';
        }
        if (output.showbase) { write('0'); write('b'); }
      } else
      if (base == HEX) {
        if (strlen(str) & 1 && output.leadzero) *--str = '0';
        if (output.showbase) { write('0'); write('x'); }
      } else
      if (base == OCT) {
        if (output.showbase) { write('0'); }
      }
      print(str);
    }

    void printFloat(double number, int digits) {
      if (isnan(number)) return print_P(PSTR("nan"));
      if (isinf(number)) return print_P(PSTR("infinite"));
      if (number >  4294967040.0) return print_P(PSTR("ovf")); // constant determined empirically
      if (number < -4294967040.0) return print_P(PSTR("ovf")); // constant determined empirically

      // handle negative numbers
      if (number < 0.0) {
        print('-');
        number = -number;
      }

      // round correctly so that print(1.999, 2) prints as "2.00"
      double rounding = 0.5;
      for (int i = 0; i < digits; i++) rounding /= 10.0;
      number += rounding;

      // extract the integer part of the number and print it
      unsigned long int_part = (unsigned long)number;
      double remainder = number - (double)int_part;
      print(int_part);

      // print the decimal point, but only if there are digits beyond
      if (digits > 0) print('.');

      // extract digits from the remainder one at a time
      while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int rem = (unsigned int)remainder;
        print(rem);
        remainder -= rem;
      }
    }

};

/*
inline nPrint& dec(nPrint& p)
{ p.setBase(DEC); return p; }

inline nPrint& hex(nPrint& p)
{ p.setBase(HEX); return p; }

inline nPrint& bin(nPrint& p)
{ p.setBase(BIN); return p; }

inline nPrint& oct(nPrint& p)
{ p.setBase(OCT); return p; }
*/

inline nPrint& operator<<(nPrint& p, const _baseint& n)
{ p.setOutputbase(n); return p; }

//inline nPrint& endl(nPrint& p)
//{ p.setBase(DEC); p.println(); return p; }

inline nPrint& operator<<(nPrint& p, const _endline& n)
{ p.println(); return p; }

#endif //Print_h