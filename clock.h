// A replacement for the Arduino system clock with the following improvements:
// - Uses Timer2 for asynchronous operation
// - 32kHz crystal support
// - Interrupt-driven timeouts
#ifndef __CF_CLOCK_H__
#define __CF_CLOCK_H__

// We should not be using MiniCore for a stock Uno
#ifdef MINICORE
  #define CLOCK_USE_32KHZ_CRYSTAL
#endif

#ifdef CLOCK_USE_32KHZ_CRYSTAL
  #define CLOCK_TICKS_PER_SECOND 32768UL
#elif F_CPU == 1000000
  // A 1 MHz clock is an exception since we don't have the right prescaler value available to
  // make it match the rest of the available clock speeds
  #define CLOCK_TICKS_PER_SECOND 31250UL
#else
  #define CLOCK_TICKS_PER_SECOND 62500UL
#endif

#define CLOCK_TICKS_TO_MICROS(t) ((unsigned long) ((t) * 1e6 / CLOCK_TICKS_PER_SECOND + 0.5))
#define CLOCK_MICROS_TO_TICKS(m) ((unsigned long) ((m) * (CLOCK_TICKS_PER_SECOND / 1e6) + 0.5))



namespace clock {

// Must be called first before calling any other function
void start();

// This must be called before entering power save mode. It should also be called before calling
// ticks() after waking from power save mode to ensure complete accuracy. See
// "Asynchronous Operation of Timer/Counter2" in the ATmega328P datasheet for more details.
void waitForSync();

void setTimeout(void (*cb)(), unsigned long expirationTime);
void clearTimeout();
unsigned long ticks();

} // namespace clock

#endif
