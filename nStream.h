/*
 * nanoStream class with standard Input Output methods and w/o String object support
 *
 */
#ifndef nStream_h
#define nStream_h
#include "nPrint.h"

class nStream : public nPrint {

  public:
    virtual uint8_t read();
    virtual uint8_t peek();

    uint8_t readBytes(char *buffer, uint8_t length) { return read(buffer, length); }

    uint8_t readBytesUntil(char terminator, char *buffer, uint8_t length) { return readUntil(buffer, length, terminator); }

    inline uint16_t getTimeout() { return time_out; }

    inline void setTimeout(uint16_t timeout = 2000) {
      time_out = timeout;
    }

    // buffer is NOT null terminated, if need null at the end, buffer length must be length + 1
    uint8_t read(char *buffer, uint8_t length = 0) {
      if (buffer == NULL) return 0;
      uint8_t count = length, c = 0;
      while (count && (c = timedRead())) {
        *buffer++ = c;
        count--;
      }
      return length - count;
    }

    // terminates if length characters have been read, timeout, or if the terminator character detected
    uint8_t readUntil(char *buffer, uint8_t length = 0, char terminator = 13) {
      if (buffer == NULL) return 0;
      uint8_t count = length, c = 0;
      while (count && (c = timedRead())) {
        if (c == terminator) break;
        *buffer++ = c;
        count--;
      }
      return length - count;
    }

    // returns true if target string is found, false if timed out
    inline bool find(const char *target) {
      //if (target == NULL) return false;
      MultiTarget t[1] = {target, (uint8_t)strlen(target), 0};
      return findMulti(t, 1) == 0 ? true : false;
    }

    // returns true if target string is found, false if timed out or terminated
    // Note: if reti = true, return 1 is meaning target found, but
    //      if reti = false, return 0 is meaning target found
    inline int8_t findUntil(const char *target, const char *terminator = NULL, bool reti = true) {
      //if (target     == NULL) return false;
      if (terminator == NULL) return false;
      MultiTarget t[2] = {target, (uint8_t)strlen(target), 0, terminator, (uint8_t)strlen(terminator), 0};
      if (reti) return findMulti(t, 2) == 0 ? true : false;
      return findMulti(t, 2);
    }

    inline bool find_P(const char *target) {
      //if (target == NULL) return false;
      MultiTarget t[1] = {target, (uint8_t)strlen_P(target), 0};
      return findMulti_P(t, 1) == 0 ? true : false;
    }

    inline int8_t findUntil_P(const char *target, const char *terminator = NULL, bool reti = true) {
      //if (target     == NULL) return false;
      if (terminator == NULL) terminator = PSTR("OK");
      MultiTarget t[2] = {target, (uint8_t)strlen_P(target), 0, terminator, (uint8_t)strlen_P(terminator), 0};
      if (reti) return findMulti_P(t, 2) == 0 ? true : false;
      return findMulti_P(t, 2);      // -1 time out; 0 target found; 1 terminator found
    }

    inline bool send_P(const nPrint& cmd, const char *target = NULL, bool stop = false) {
      return response_P(target, stop);
    }

    inline bool send_P(const char *cmd, const char *target = NULL, bool stop = false) {
      delay(50);
	  println_P(cmd);
      return response_P(target, stop);
    }

    uint8_t timedRead() {
      uint32_t prev_millis = millis();
      do {
        if (peek()) return read();
      } while (millis() - prev_millis < time_out);
      return 0;               // indicates timeout
    }

    uint8_t timedPeek() {
      uint32_t prev_millis = millis();
      do {
        if (peek()) return peek();
      } while (millis() - prev_millis < time_out);
      return 0;               // indicates timeout
    }

  protected:
    uint16_t time_out = 2000; // max is 65535 of milliseconds to wait for the next char in timedRead

    struct MultiTarget {
      const char *str;        // string you're searching for
      uint8_t len;            // length of string you're searching for
      uint8_t index;          // index used by the search routine
    };

    bool response_P(const char *target, bool stop) {
      if (target) {
        if (stop) return find_P(target);
        else return find_P(target) ? find_P(PSTR("OK")) : false;
      }
      else return find_P(PSTR("OK"));
    }

    // findMulti returns: 0 target found, 1 terminator found, -1 time out
    int8_t findMulti(MultiTarget *targets, uint8_t tCount)
    { // any zero length targets string automatically matches
      //for (MultiTarget *t = targets; t < targets + tCount; ++t) {
      //if (t->len <= 0) return t - targets;
      //}
      while (uint8_t c = timedRead())
      {
        for (MultiTarget *t = targets; t < targets + tCount; ++t) {
          // the simple case is if we match, deal with that first
          if (c == t->str[t->index]) {
            if (++t->index == t->len) return t - targets;
            else continue;
          }
          // if not we need to walk back and see if we could have matched further down the stream
          // (ie '1112' doesn't match the first position in '11112') but it will match the second position
          // so we can't just reset the current index to 0 when we find a mismatch
          if (t->index == 0) continue;

          uint8_t origIndex = t->index;
          do {
            --t->index;
            // first check if current char works against the new current index
            if (c != t->str[t->index]) continue;
            // if it's the only char then we're good, nothing more to check
            if (t->index == 0) {
              t->index++;
              break;
            }
            // otherwise we need to check the rest of the found string
            uint8_t i = 0, diff = origIndex - t->index;
            for (; i < t->index; ++i) {
              if (t->str[i] != t->str[i + diff]) break;
            }
            // if we successfully got through the previous loop then our current index is good
            if (i == t->index) {
              t->index++;
              break;
            }
            // otherwise we just try the next index
          } while (t->index);
        }
      }
      return -1; // indicates timeout
    }

    // compare Serial byte with Progmem byte
    int8_t findMulti_P(MultiTarget *targets, uint8_t tCount)
    { // any zero length targets string automatically matches
      //for (MultiTarget *t = targets; t < targets + tCount; ++t) {
      //if (t->len <= 0) return t - targets;
      //}
      while (uint8_t c = timedRead())
      {
        for (MultiTarget *t = targets; t < targets + tCount; ++t) {
          // the simple case is if we match, deal with that first
          if (c == pgm_read_byte(t->str + t->index)) {
            if (++t->index == t->len) return t - targets;
            else continue;
          }
          // if not we need to walk back and see if we could have matched further down the stream
          // (ie '1112' doesn't match the first position in '11112') but it will match the second position
          // so we can't just reset the current index to 0 when we find a mismatch
          if (t->index == 0) continue;

          uint8_t origIndex = t->index;
          do {
            --t->index;
            // first check if current char works against the new current index
            if (c != pgm_read_byte(t->str + t->index)) continue;
            // if it's the only char then we're good, nothing more to check
            if (t->index == 0) {
              t->index++;
              break;
            }
            // otherwise we need to check the rest of the found string
            uint8_t i = 0, diff = origIndex - t->index;
            for (; i < t->index; ++i) {
              if (pgm_read_byte(t->str + i) != pgm_read_byte(t->str + i + diff)) break;
            }
            // if we successfully got through the previous loop then our current index is good
            if (i == t->index) {
              t->index++;
              break;
            }
            // otherwise we just try the next index
          } while (t->index);
        }
      }
      return -1; // indicates timeout
    }

};
#endif //Stream_h