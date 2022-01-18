#include <Arduino.h>

#include "clock.h"
#include "debug.h"


namespace {

void (*callback)() = nullptr;
unsigned long timeout = 0UL;
unsigned long microsPerTick = 0UL; // 0 indicates not yet initialized
volatile unsigned long nextCycleMicros = 0UL;
volatile unsigned long microsElapsed = 0UL;
volatile bool callbackCalled = false;



void clockInit() {
  unsigned char prescaler = 0;

  #if F_CPU == 8000000
    // 8 micros precision (64 / 8 MHz)
    microsPerTick = 8UL;
    prescaler = bit(CS22);
  #elif F_CPU == 4000000
    // 8 micros precision (32 / 4 MHz)
    microsPerTick = 8UL;
    prescaler = bit(CS21) | bit(CS20);
  #elif F_CPU == 2000000
    // 16 micros precision (32 / 2 MHz)
    microsPerTick = 16UL;
    prescaler = bit(CS21) | bit(CS20);
  #elif F_CPU == 1000000
    // 8 micros precision (8 / 1 MHz)
    microsPerTick = 8UL;
    prescaler = bit(CS21);
  #else
    // Assume 16 MHz
    // 8 micros precision (128 / 16 MHz)
    microsPerTick = 8UL;
    prescaler = bit(CS22) | bit(CS20);
  #endif

  nextCycleMicros = 256 * microsPerTick;

  cli();

  ASSR &= ~bit(AS2); // Internal clock
  TCCR2A = 0;	// Normal operation
  TCCR2B = prescaler;
  TCNT2 = 0;

  // Wait for registers to update
  while (ASSR & (bit(TCR2BUB) | bit(TCR2AUB) | bit(TCN2UB) | bit(OCR2AUB)));

  // Reset prescaler and wait for it to finish
  GTCCR |= bit(PSRASY);
  while (GTCCR & bit(PSRASY));

  sei();

  DEBUG_PRINTLN("Timer2 initialized");
}

} // namespace



// Timer2 overflow interrupt routine
ISR(TIMER2_OVF_vect) {
  microsElapsed += nextCycleMicros;

  if (!callbackCalled) {
    if ((long) (microsElapsed - timeout) >= 0) {
      // Reset the cycle size in case we had a remainder on the final cycle
      nextCycleMicros = 256 * microsPerTick;

      callback();
      callbackCalled = true;
    }
    else if (timeout - microsElapsed < nextCycleMicros) {
      // Count only the remaining micros until the timeout on the next cycle
      nextCycleMicros = timeout - microsElapsed;
      TCNT2 = (unsigned char) (256UL - nextCycleMicros / microsPerTick);
    }
  }
}



namespace clock {

void setTimeout(void (*cb)(), unsigned long durationMicros) {
  if (microsPerTick == 0UL) {
    // First call, run clockInit() which will set everything up
    clockInit();
  }

  timeout = micros() + durationMicros;
  // We have a maximum precision of microsPerTick, so we call the callback at the *beginning*
  // of the tick that contains the timeout. Round the timeout down to the nearest tick to reflect
  // this.
  timeout -= timeout % microsPerTick;

  callback = cb;
  callbackCalled = false;
  TIMSK2 = bit(TOIE2); // Enable overflow interruption when 0
}

void clearTimeout() {
  callbackCalled = true;
  callback = nullptr;
}

unsigned long micros() {
  unsigned long sinceLastCycle = TCNT2 * microsPerTick;

  // Special case for truncated final tick, we skipped the part of this cycle
  if (nextCycleMicros < 256 * microsPerTick) {
    sinceLastCycle -= 256 * microsPerTick - nextCycleMicros;
  }

  return microsElapsed + sinceLastCycle;
}

void stop() {
  // Disable timer2 interrupts
  TIMSK2 = 0;
}

} // namespace clock
