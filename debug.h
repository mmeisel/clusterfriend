#ifndef DEBUG_H
#define DEBUG_H

// Global debug mode setting
#define DEBUG 1

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_FLUSH() Serial.flush()
#else
#define DEBUG_PRINT(...) do {} while (false)
#define DEBUG_PRINTLN(...) do {} while (false)
#define DEBUG_FLUSH() do {} while (false)
#endif

#endif
