#ifndef __CF_CLOCK_H__
#define __CF_CLOCK_H__

namespace clock {
  void set_timeout(void (*cb)(), unsigned long duration_micros);
  void clear_timeout();
  unsigned long micros();
  void stop();
}

#endif
