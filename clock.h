#ifndef __CF_CLOCK_H__
#define __CF_CLOCK_H__

namespace clock {

// Must be called first before calling any other function
void start();

void setTimeout(void (*cb)(), unsigned long durationMicros);
void clearTimeout();
unsigned long micros();

} // namespace clock

#endif
