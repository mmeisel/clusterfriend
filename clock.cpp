#include <Arduino.h>

#include "clock.h"
#include "debug.h"

#if F_CPU == 8000000
  // 8 micros precision (64 / 8 MHz)
  #define CLOCK_MICROS_PER_TICK 8UL
  #define CLOCK_PRESCALER (bit(CS22))
#elif F_CPU == 4000000
  // 8 micros precision (32 / 4 MHz)
  #define CLOCK_MICROS_PER_TICK 8UL
  #define CLOCK_PRESCALER (bit(CS21) | bit(CS20))
#elif F_CPU == 2000000
  // 16 micros precision (32 / 2 MHz)
  #define CLOCK_MICROS_PER_TICK 16UL
  #define CLOCK_PRESCALER (bit(CS21) | bit(CS20))
#elif F_CPU == 1000000
  // 8 micros precision (8 / 1 MHz)
  #define CLOCK_MICROS_PER_TICK 8UL
  #define CLOCK_PRESCALER (bit(CS21))
#else
  // Assume 16 MHz
  // 8 micros precision (128 / 16 MHz)
  #define CLOCK_MICROS_PER_TICK 8UL
  #define CLOCK_PRESCALER (bit(CS22) | bit(CS20))
#endif

#define CLOCK_MICROS_PER_OVERFLOW (256UL * CLOCK_MICROS_PER_TICK)



namespace {

void (*callback)() = nullptr;
unsigned long timeout = 0UL;
volatile unsigned long nextCycleMicros = 0UL;
volatile unsigned long microsElapsed = 0UL;
volatile bool timeoutActive = false;

} // namespace



// Timer2 overflow interrupt routine
ISR(TIMER2_OVF_vect) {
  microsElapsed += nextCycleMicros;

  if (timeoutActive) {
    long remaining = timeout - microsElapsed;

    if (remaining <= 0) {
      // Reset the cycle size in case we had a remainder on the final cycle
      nextCycleMicros = CLOCK_MICROS_PER_OVERFLOW;

      callback();
      timeoutActive = false;
    }
    else if ((unsigned long) remaining < CLOCK_MICROS_PER_OVERFLOW) {
      // Count only the remaining micros until the timeout on the next cycle
      nextCycleMicros = remaining;
      TCNT2 = (unsigned char) (256L - remaining / CLOCK_MICROS_PER_TICK);
    }
  }
}



namespace clock {

void start() {
  cli();

  ASSR &= ~bit(AS2); // Internal clock
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

  nextCycleMicros = CLOCK_MICROS_PER_OVERFLOW;

  DEBUG_PRINT(F("CLOCK "));
  DEBUG_PRINT(CLOCK_MICROS_PER_TICK);
  DEBUG_PRINT(F(" micros per tick @ "));
  DEBUG_PRINT(F_CPU / 1000000);
  DEBUG_PRINTLN(F(" MHz"));
}



void setTimeout(void (*cb)(), unsigned long durationMicros) {
  timeout = clock::micros() + durationMicros;
  // We have a maximum precision of CLOCK_MICROS_PER_TICK, so we call the callback at the *beginning*
  // of the tick that contains the timeout. Round the timeout down to the nearest tick to reflect
  // this.
  timeout -= timeout % CLOCK_MICROS_PER_TICK;

  callback = cb;
  timeoutActive = true;
}



void clearTimeout() {
  timeoutActive = false;
  callback = nullptr;
}



unsigned long micros() {
  unsigned long sinceLastCycle = TCNT2 * CLOCK_MICROS_PER_TICK;

  // Special case for truncated final tick, we skipped part of this cycle
  if (nextCycleMicros < CLOCK_MICROS_PER_OVERFLOW) {
    sinceLastCycle -= CLOCK_MICROS_PER_OVERFLOW - nextCycleMicros;
  }

  return microsElapsed + sinceLastCycle;
}

} // namespace clock
