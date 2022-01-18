// Synchronization algorithm:
//
// The concept here is for new nodes to add themselves to the end of an already synchronized group.
//
// Nodes broadcast in slots on a fixed cycle. Though the cycle duration is relatively long, the
// nodes all try to pack in to the beginning of the cycle, sleeping for the rest of it. When a new
// node wants to join, it waits and listens until it hears other nodes. As part of their broadcast,
// nodes announce their slot number. A new node takes the highest slot number it hears and uses the
// next one. So if the highest slot it heard used was slot 9, it adds a tenth slot, announcing
// itself at slot 10 in the next cycle.
//
// If a node hears no other nodes in its initial listening period, it starts its own cycle,
// broadcasting at slot 0.
//
// If two nodes are trying to transmit in the same slot (which can happen, e.g., if two nodes are
// trying to join the network at the same time), whoever transmits first "wins." The other node will
// cease transmission in the current cycle and rejoin the network at the end of the pack in the next
// cycle.
//
// In order to recover wasted slots (e.g. if a node leaves the network), nodes track how many cycles
// in a row the slot below them goes unused. If this number reaches a certain (configurable)
// threshold, the node will "steal" the slot below it. For example, if a node is broadcasting at
// slot 4, but hears slot 3 go unused for N cycles in a row, it will switch to slot 3. In the same
// way, all nodes also track the use of the final slot, using the same threshold to decide when to
// reduce their expected total number of nodes in the network. This lets the nodes end their cycle
// (and therefore sleep their radio) a bit sooner, thus completing the slot recovery process.
//
#include <Arduino.h>

#include "clock.h"
#include "debug.h"
#include "packet.h"
#include "tdma.h"



#define TDMA_CYCLE_DURATION 5000000UL // 5 seconds in microseconds
#define TDMA_SLOT_PADDING 30000UL // How much silence to allow in each slot, 30 ms in microseconds
#define TDMA_STEAL_AFTER_UNUSED_CYCLES 6 // How long to wait to reuse slots, max 255



// Implementation details
namespace {

using namespace tdma;

// Provided/computed at startup time
unsigned long packetAirtimeMicros = 0UL;
unsigned long slotSize = 0UL;

// TDMA state
volatile unsigned long cycleStartTime = 0UL;
volatile int totalSlots = 0;
volatile int mySlot = -1;
volatile bool startupPhase = true;
volatile State myState = State::cycleComplete;
volatile bool timeoutFired = false;
int cycleNeighborCount = 0;
long cycleOffset = 0L;
uint8_t prevSlotUnusedCycles = 0;
uint8_t lastSlotUnusedCycles = 0;



void handleCycleStartTimeout() {
  // We always wake up one slotSize early
  cycleStartTime = clock::micros() + slotSize;
  myState = State::cycleStarted;
  timeoutFired = true;
}



void handleCycleEndTimeout() {
  myState = State::cycleComplete;
  timeoutFired = true;
}



void handleTransmitTimeout() {
  myState = State::txReady;
  timeoutFired = true;
}



void handleStartupTimeout() {
  // All alone
  DEBUG_PRINT(clock::micros());
  DEBUG_PRINTLN(F(" TDMA alone"));

  mySlot = 0;
  totalSlots = 1;
  startupPhase = false;
  handleCycleStartTimeout();
}



void setCycleStartTimeout() {
  // Adjust our cycle start time based on the average discrepancy between the expected start time
  // for each slot versus the actual receive time, including ourselves in the average by adding
  // one to the denominator (from our perspective, our offset of course is zero).
  long adjustment = cycleOffset / (cycleNeighborCount + 1);

  // Wake up a little early to make sure we hear (or properly schedule) the first slot.
  unsigned long cycleStartTimeout = TDMA_CYCLE_DURATION - slotSize + adjustment - (
    clock::micros() - cycleStartTime
  );

  clock::setTimeout(handleCycleStartTimeout, cycleStartTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA sleep for "));
  DEBUG_PRINT(cycleStartTimeout);
  DEBUG_PRINT(F(" adjustment="));
  DEBUG_PRINTLN(adjustment);
}



void setCycleEndTimeout() {
  // Listen for a couple extra slots in case someone new is trying to join
  unsigned long cycleEndTimeout = slotSize * (totalSlots + 2) - (
    clock::micros() - cycleStartTime
  );
  clock::setTimeout(handleCycleEndTimeout, cycleEndTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA cycle ends in "));
  DEBUG_PRINTLN(cycleEndTimeout);
}



void setTransmitTimeout() {
  unsigned long transmitTimeout = slotSize * mySlot;
  unsigned long now = clock::micros();

  // We wake up a little before the first slot, so the cycleStartTime may be in the future.
  if ((long) (cycleStartTime - now) > 0) {
    transmitTimeout += cycleStartTime - now;
  }
  else {
    transmitTimeout -= now - cycleStartTime;
  }

  clock::setTimeout(handleTransmitTimeout, transmitTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" Will transmit in "));
  DEBUG_PRINTLN(transmitTimeout);
}



void startCycle() {
  cycleNeighborCount = 0;
  cycleOffset = 0L;

  if (mySlot > 0) {
    prevSlotUnusedCycles++;
  }

  if (mySlot != totalSlots - 1) {
    lastSlotUnusedCycles++;
  }

  setTransmitTimeout();

  DEBUG_PRINT(F("<-> "));
  DEBUG_PRINTLN(cycleStartTime);
}



void endCycle() {
  if (startupPhase) {
    // We heard a cycle during the startup phase! End the startup phase and pick a slot.
    startupPhase = false;
    mySlot = totalSlots;
    totalSlots++;
    prevSlotUnusedCycles = 0;
    lastSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA join | slot="));
    DEBUG_PRINT(mySlot);
    DEBUG_PRINT(F(" total="));
    DEBUG_PRINTLN(totalSlots);
  }
  else if (mySlot > 0 && prevSlotUnusedCycles >= TDMA_STEAL_AFTER_UNUSED_CYCLES) {
    // The slot below us isn't in use, steal it to defragment the cycle. If we were in the last
    // slot, reduce our total slots (everyone else will figure it out eventually).
    if (mySlot == totalSlots - 1) {
      totalSlots--;
    }
    mySlot--;
    prevSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA my slot now "));
    DEBUG_PRINTLN(mySlot);
  }

  if (
    mySlot < totalSlots - 1 &&
    lastSlotUnusedCycles >= TDMA_STEAL_AFTER_UNUSED_CYCLES
  ) {
    // The last slot is unused, reduce our total slots to reduce the overall cycle time.
    totalSlots--;
    lastSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA total slots decreased to "));
    DEBUG_PRINTLN(totalSlots);
  }

  setCycleStartTimeout();
}

} // namespace



namespace tdma {

void startup(unsigned long packetAirtime) {
  packetAirtimeMicros = packetAirtime;
  slotSize = packetAirtimeMicros + TDMA_SLOT_PADDING;
  // Initial startup phase timer
  clock::setTimeout(handleStartupTimeout, TDMA_CYCLE_DURATION * 3);
}



void processPacket(const packet::Packet& packet, unsigned long receiveTime) {
  // TODO: do we need to worry about transmission delay?
  unsigned long neighborCycleStartTime = (
    receiveTime -
    packet.delayMillis * 1000UL -
    packetAirtimeMicros -
    packet.slot * slotSize
  );

  if (startupPhase && cycleStartTime == 0UL) {
    // This is the first neighbor we heard in the startup phase! We don't have a cycleStartTime
    // yet, so make a first approximation here. We'll make a finer adjustment at the end of the
    // cycle, just like in all future cycles.
    cycleStartTime = neighborCycleStartTime;
  }

  // Track the discrepancy between the expected start time for each slot versus the actual
  // receive time. This will be used to keep us in sync with our neighbors.
  cycleNeighborCount++;
  cycleOffset += (long) (neighborCycleStartTime - cycleStartTime);

  if (packet.slot == mySlot - 1) {
    // Track when the previous slot is unusued so we can steal it
    prevSlotUnusedCycles = 0;
  }
  else if (packet.slot == mySlot) {
    // Uh oh, we have a conflict!
    // If we haven't already transmitted, cancel transmission and pick a new slot when the cycle
    // completes -- essentially leaving and re-joining the network.
    if (myState == State::cycleStarted || myState == State::txReady) {
      startupPhase = true;
      setCycleEndTimeout();
    }
  }
  else if (packet.slot == totalSlots - 1) {
    // Track when the last slot is unused so we can shrink the cycle accordingly
    lastSlotUnusedCycles = 0;
  }
  else if (packet.slot >= totalSlots) {
    // A new neighbor! Update our state and reset our end of cycle timer if necesssary.
    totalSlots = packet.slot + 1;
    lastSlotUnusedCycles = 0;

    if (startupPhase || myState == State::txComplete) {
      // We already transmitted or are still starting up, so reschedule our end-of-cycle sleep.
      // (This should always be the case unless the new neighbor transmitted way too early.)
      setCycleEndTimeout();
    }

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA total slots increased to "));
    DEBUG_PRINTLN(totalSlots);
  }
}



void txComplete() {
  myState = State::txComplete;
  setCycleEndTimeout();
}



State checkState() {
  if (timeoutFired) {
    timeoutFired = false;

    if (myState == State::cycleStarted) {
      startCycle();
    }
    else if (myState == State::cycleComplete) {
      endCycle();
    }
  }

  return myState;
}



int getSlotNumber() {
  return mySlot;
}



long getSlotTimeElapsed() {
  return clock::micros() - mySlot * slotSize - cycleStartTime;
}

} // namespace tdma
