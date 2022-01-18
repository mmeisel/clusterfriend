#ifndef __CF_TDMA_H__
#define __CF_TDMA_H__

#include "packet.h"

namespace tdma {

enum class State:uint8_t {
  cycleStarted,
  txReady,
  txComplete,
  cycleComplete
};

// Call from setup()
void startup(unsigned long packetAirtime);

// Must be called unconditionally from loop()! Returns the current state. Use this to determine
// when to listen (cycleStarted), when to transmit (txReady), and when to sleep (cycleComplete).
State checkState();

// Call when a packet arrives
void processPacket(const packet::Packet& packet, unsigned long receiveTime);

// Call when done transmitting
void txComplete();

// This node's slot number (zero-indexed), -1 means still starting up
int getSlotNumber();

// How many microseconds have elapsed since the beginning of the node's slot in the current cycle.
// Signed to give a valid value before the start of the node's slot.
long getSlotTimeElapsed();

} // namespace tdma

#endif
