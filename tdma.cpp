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
// trying to join the network at the same time), each flips a coin to see if they will vacate the
// slot and rejoin the network at the end of the pack in the next cycle.
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



#define TDMA_CYCLE_DURATION (2UL * CLOCK_TICKS_PER_SECOND) // 2 seconds in ticks
#define TDMA_SLOT_PADDING (CLOCK_TICKS_PER_SECOND / 32UL) // How much silence to allow in a slot, ~31 ms
#define TDMA_STEAL_AFTER_UNUSED_CYCLES 6 // How long to wait to reuse slots, max 255
#define TDMA_MAX_TX_DELAY_TICKS (TDMA_SLOT_PADDING / 5UL) // Don't delay more than 20% of padding



// Implementation details
namespace {

using namespace tdma;

// Provided/computed at startup time
unsigned long packetAirtimeTicks = 0UL;
unsigned long slotSize = 0UL;
int availableSlots = 0;

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
  // We always wake up one TDMA_SLOT_PADDING early
  cycleStartTime = clock::ticks() + TDMA_SLOT_PADDING;
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
  DEBUG_PRINT(clock::ticks());
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
  unsigned long wakeUpTime = cycleStartTime + TDMA_CYCLE_DURATION - TDMA_SLOT_PADDING + adjustment;

  clock::setTimeout(handleCycleStartTimeout, wakeUpTime);

  DEBUG_PRINT(clock::ticks());
  DEBUG_PRINT(F(" TDMA sleep until "));
  DEBUG_PRINT(wakeUpTime);
  DEBUG_PRINT(F(" adjustment:"));
  DEBUG_PRINTLN(adjustment);
}



void setCycleEndTimeout() {
  // Listen for a couple extra slots in case someone new is trying to join
  unsigned long cycleEndTime = cycleStartTime + slotSize * (totalSlots + 2);

  clock::setTimeout(handleCycleEndTimeout, cycleEndTime);

  DEBUG_PRINT(clock::ticks());
  DEBUG_PRINT(F(" TDMA cycle ends at "));
  DEBUG_PRINTLN(cycleEndTime);
}



void setTransmitTimeout() {
  // Add a small random offset from the start of the slot to help resolve any collisions.
  // The packets we send announce this delay so it won't affect synchronization.
  unsigned long txTime = cycleStartTime + slotSize * mySlot + random(TDMA_MAX_TX_DELAY_TICKS);

  clock::setTimeout(handleTransmitTimeout, txTime);

  DEBUG_PRINT(clock::ticks());
  DEBUG_PRINT(F(" TDMA will transmit at "));
  DEBUG_PRINTLN(txTime);
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

    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" TDMA join at "));
    DEBUG_PRINTLN(mySlot);
  }
  else if (mySlot > 0 && prevSlotUnusedCycles >= TDMA_STEAL_AFTER_UNUSED_CYCLES) {
    // The slot below us isn't in use, steal it to defragment the cycle. If we were in the last
    // slot, reduce our total slots (everyone else will figure it out eventually).
    if (mySlot == totalSlots - 1) {
      totalSlots--;
    }
    mySlot--;
    prevSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::ticks());
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

    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" TDMA total slots decreased to "));
    DEBUG_PRINTLN(totalSlots);
  }

  setCycleStartTimeout();
}

} // namespace



namespace tdma {

void start(unsigned long packetAirtime) {
  packetAirtimeTicks = packetAirtime;
  slotSize = packetAirtimeTicks + TDMA_SLOT_PADDING;
  availableSlots = TDMA_CYCLE_DURATION / slotSize;
  // Initial startup phase timer
  clock::setTimeout(handleStartupTimeout, clock::ticks() + TDMA_CYCLE_DURATION * 3);
}



void processPacket(const packet::Packet& packet, unsigned long receiveTime) {
  uint8_t slot = packet.getSlot();

  if (slot >= availableSlots) {
    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" TDMA no such slot! "));
    DEBUG_PRINTLN(slot);
    return;
  }

  // TODO: do we need to worry about transmission delay?
  unsigned long receivedDelay = packet.getDelayTicks();
  unsigned long neighborCycleStartTime = (
    receiveTime -
    receivedDelay -
    packetAirtimeTicks -
    slot * slotSize
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

  if (slot == mySlot - 1) {
    // Track when the previous slot is unusued so we can steal it
    prevSlotUnusedCycles = 0;
  }
  else if (slot == mySlot) {
    // Uh oh, we have a conflict!
    // Flip a coin to decide whether to keep the slot or pick a new slot when the cycle completes,
    // essentially leaving and re-joining the network.
    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" TDMA conflict"));

    if (random(2) == 0) {
      startupPhase = true;
      DEBUG_PRINTLN(F(" move"));
    }
    else {
      DEBUG_PRINTLN(F(" stay"));
    }
  }
  else if (slot == totalSlots - 1) {
    // Track when the last slot is unused so we can shrink the cycle accordingly
    lastSlotUnusedCycles = 0;
  }
  else if (slot >= totalSlots) {
    // A new neighbor! Update our state and reset our end of cycle timer if necesssary.
    totalSlots = slot + 1;
    lastSlotUnusedCycles = 0;

    if (startupPhase || myState == State::txComplete) {
      // We already transmitted or are still starting up, so reschedule our end-of-cycle sleep.
      // (This should always be the case unless the new neighbor transmitted way too early.)
      setCycleEndTimeout();
    }

    DEBUG_PRINT(clock::ticks());
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



long getSlotTicksElapsed() {
  return clock::ticks() - mySlot * slotSize - cycleStartTime;
}

} // namespace tdma
