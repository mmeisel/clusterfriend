#ifndef __CF_CLOCK_H__
#define __CF_CLOCK_H__

namespace clock {
  void set_timeout(void (*cb)(), unsigned long micros);
  void stop();
  unsigned long elapsed();
}

#endif
