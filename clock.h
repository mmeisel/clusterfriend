#ifndef __CF_CLOCK_H__
#define __CF_CLOCK_H__

namespace clock {

void setTimeout(void (*cb)(), unsigned long durationMicros);
void clearTimeout();
unsigned long micros();
void stop();

} // namespace clock

#endif
