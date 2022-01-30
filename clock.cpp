#include <Arduino.h>

#include "clock.h"
#include "debug.h"

#ifdef CLOCK_USE_32KHZ_CRYSTAL
  // No prescaling, ~30.5 micros precision
  #define CLOCK_PRESCALER bit(CS20)
#elif F_CPU == 16000000
  // 16 micros precision (256 / 16 MHz)
  #define CLOCK_PRESCALER (bit(CS22) | bit(CS21))
#elif F_CPU == 8000000
  // 16 micros precision (128 / 8 MHz)
  #define CLOCK_PRESCALER (bit(CS22) | bit(CS20))
#elif F_CPU == 4000000
  // 16 micros precision (64 / 4 MHz)
  #define CLOCK_PRESCALER bit(CS22)
#else
  // 2MHz or less. Either 16 micros (32 / 2 MHz) or 32 micros (32 / 1MHz) precision
  #define CLOCK_PRESCALER (bit(CS21) | bit(CS20))
#endif



namespace {

void (*callback)() = nullptr;
unsigned long timeout = 0UL;
volatile unsigned int nextCycleTicks = 256;
volatile unsigned long currentTime = 0UL;
volatile bool timeoutActive = false;

} // namespace



// Timer2 overflow interrupt routine
ISR(TIMER2_OVF_vect) {
  currentTime += nextCycleTicks;

  if (timeoutActive) {
    long remaining = timeout - currentTime;

    if (remaining <= 0) {
      // Reset the cycle size in case we had a remainder on the final cycle
      nextCycleTicks = 256;

      callback();
      timeoutActive = false;
    }
    else if (remaining < 256L) {
      // Count only the remaining ticks until the timeout on the next cycle
      nextCycleTicks = remaining;
      TCNT2 = (unsigned char) (256L - remaining);
    }
  }
}



namespace clock {

void start() {
  cli();

  ASSR |= bit(AS2); // Asynchronous clock so it keeps running in power save mode
  TCCR2A = 0;	// Normal operation
  TCCR2B = CLOCK_PRESCALER;
  TCNT2 = 0;
  TIMSK2 = bit(TOIE2); // Enable overflow interruption when 0

  // Wait for registers to update
  while (ASSR & (bit(TCR2BUB) | bit(TCR2AUB) | bit(TCN2UB) | bit(OCR2AUB)));

  // Reset prescaler and wait for it to finish
  GTCCR |= bit(PSRASY);
  while (GTCCR & bit(PSRASY));

  sei();

#ifdef CLOCK_USE_32KHZ_CRYSTAL
  // Give the crystal a chance to settle
  delay(5000);
#endif

  DEBUG_PRINT(F("CLOCK started at "));
  DEBUG_PRINT(CLOCK_TICKS_PER_SECOND);
  DEBUG_PRINTLN(F(" ticks per second"));
}



void setTimeout(void (*cb)(), unsigned long durationTicks) {
  timeout = ticks() + durationTicks;
  callback = cb;
  timeoutActive = true;
}



void clearTimeout() {
  timeoutActive = false;
  callback = nullptr;
}



unsigned long ticks() {
  unsigned long sinceLastCycle = TCNT2;

  // Special case for truncated final tick, we skipped part of this cycle
  if (nextCycleTicks < 256) {
    sinceLastCycle -= 256 - nextCycleTicks;
  }

  return currentTime + sinceLastCycle;
}

} // namespace clock
