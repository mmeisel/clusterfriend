#include <Arduino.h>

#include "clock.h"
#include "debug.h"


void (*callback)() = nullptr;
unsigned long timeout = 0UL;
unsigned long micros_per_tick = 0UL; // 0 indicates not yet initialized
volatile unsigned long next_cycle_micros = 0UL;
volatile unsigned long micros_elapsed = 0UL;
volatile bool callback_called = false;



void clock_init() {
  unsigned char prescaler = 0;

  #if F_CPU == 8000000
    // 8 micros precision (64 / 8 MHz)
    micros_per_tick = 8UL;
    prescaler = bit(CS22);
  #elif F_CPU == 4000000
    // 8 micros precision (32 / 4 MHz)
    micros_per_tick = 8UL;
    prescaler = bit(CS21) | bit(CS20);
  #elif F_CPU == 2000000
    // 16 micros precision (32 / 2 MHz)
    micros_per_tick = 16UL;
    prescaler = bit(CS21) | bit(CS20);
  #elif F_CPU == 1000000
    // 8 micros precision (8 / 1 MHz)
    micros_per_tick = 8UL;
    prescaler = bit(CS21);
  #else
    // Assume 16 MHz
    // 8 micros precision (128 / 16 MHz)
    micros_per_tick = 8UL;
    prescaler = bit(CS22) | bit(CS20);
  #endif

  next_cycle_micros = 256 * micros_per_tick;

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



// Timer2 overflow interrupt routine
ISR(TIMER2_OVF_vect) {
  micros_elapsed += next_cycle_micros;

  if (!callback_called) {
    if ((long) (micros_elapsed - timeout) >= 0) {
      // Reset the cycle size in case we had a remainder on the final cycle
      next_cycle_micros = 256 * micros_per_tick;

      callback();
      callback_called = true;
    }
    else if (timeout - micros_elapsed < next_cycle_micros) {
      // Count only the remaining micros until the timeout on the next cycle
      next_cycle_micros = timeout - micros_elapsed;
      TCNT2 = (unsigned char) (256UL - next_cycle_micros / micros_per_tick);
    }
  }
}



namespace clock {
  void set_timeout(void (*cb)(), unsigned long duration_micros) {
    if (micros_per_tick == 0UL) {
      // First call, run clock_init() which will set everything up
      clock_init();
    }

    timeout = micros() + duration_micros;
    // We have a maximum precision of micros_per_tick, so we call the callback at the *beginning*
    // of the tick that contains the timeout. Round the timeout down to the nearest tick to reflect
    // this.
    timeout -= timeout % micros_per_tick;

    callback = cb;
    callback_called = false;
    TIMSK2 = bit(TOIE2); // Enable overflow interruption when 0
  }

  void clear_timeout() {
    callback_called = true;
    callback = nullptr;
  }

  unsigned long micros() {
    unsigned long since_last_cycle = TCNT2 * micros_per_tick;

    // Special case for truncated final tick, we skipped the part of this cycle
    if (next_cycle_micros < 256 * micros_per_tick) {
      since_last_cycle -= 256 * micros_per_tick - next_cycle_micros;
    }

    return micros_elapsed + since_last_cycle;
  }

  void stop() {
    // Disable timer2 interrupts
    TIMSK2 = 0;
  }
}
