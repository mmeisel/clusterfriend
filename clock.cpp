#include <Arduino.h>
#include <util/atomic.h>

#include "clock.h"
#include "debug.h"

#ifdef CLOCK_USE_32KHZ_CRYSTAL
  // No prescaling, ~30.5 micros precision (1 / 32.768 kHz)
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

  ASSR = bit(AS2); // Asynchronous clock so it keeps running in power save mode
  TCCR2A = 0;	// Normal operation
  TCCR2B = CLOCK_PRESCALER;
  TCNT2 = 0;

  // Wait for registers to update
  while (ASSR & (bit(TCR2BUB) | bit(TCR2AUB) | bit(TCN2UB) | bit(OCR2AUB)));

  // Clear interrupt flag
  TIFR2 |= bit(TOV2);
  while (TIFR2 & bit(TOV2));

  // Reset prescaler and wait for it to finish
  GTCCR |= bit(PSRASY);
  while (GTCCR & bit(PSRASY));

  TIMSK2 = bit(TOIE2); // Enable overflow interruption when 0

  sei();

#ifdef CLOCK_USE_32KHZ_CRYSTAL
  // Give the crystal a chance to settle
  delay(5000);
#endif

  DEBUG_PRINT(F("CLOCK started at "));
  DEBUG_PRINT(CLOCK_TICKS_PER_SECOND);
  DEBUG_PRINTLN(F(" ticks per second"));
}



void waitForSync() {
  // This will wait for the internal I/O clock to be synced with the asynchronous clock. According
  // to the datasheet, we must do this:
  // - Before entering power save
  // - After waking from power save before reading TCNT2
  OCR2A = 0;
  while (ASSR & bit(OCR2AUB));
}



void setTimeout(void (*cb)(), unsigned long expirationTime) {
  // According to the datasheet, during asynchronous operation, the interrupt handler is always
  // called at least one clock cycle after the overflow. Therefore, we can be a bit more accurate
  // (without ever firing too early) by subtracting one from the expiration time.
  timeout = expirationTime - 1UL;
  callback = cb;
  timeoutActive = true;
}



void clearTimeout() {
  timeoutActive = false;
  callback = nullptr;
}



unsigned long ticks() {
  unsigned long curTicks;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Use nextCycleTicks to account for a truncated final cycle before a timeout
    curTicks = currentTime + 256 - nextCycleTicks + TCNT2;
  }
  return curTicks;
}

} // namespace clock
