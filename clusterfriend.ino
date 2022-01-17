// Required libraries:
// - Low-Power v1.81.0 (https://github.com/rocketscream/Low-Power)
// - RadioLib v5.1.0 (https://github.com/jgromes/RadioLib)
#include <LowPower.h>
#include <RadioLib.h>

#include "debug.h"
#include "clock.h"
#include "packet.h"



// Networking parameters
#define RFM_FREQ 915.0      // In MHz
#define RFM_BW 125.0        // In kHz, default 125
#define RFM_OUTPUT_POWER 10 // 2..20 dBm, 10 dBm is default
#define LORA_SF 7           // 7-12, default 9
#define LORA_PREAMBLE 8     // Default is 8 (or maybe 16?)
#define LORA_CR 7           // Translates to 4/x, default 7

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
#define TDMA_CYCLE_DURATION 5000000UL // 5 seconds in microseconds
#define TDMA_SLOT_PADDING 30000UL // How much silence to allow in each slot, 30 ms in microseconds
#define TDMA_STEAL_AFTER_UNUSED_CYCLES 6 // How long to wait to reuse slots, max 255

// Pins
#define RFM_RST_PIN 2
#define RFM_G0_PIN 3
#define RFM_CS_PIN 4
#define LED_PIN 8

// Some versions of Arduino define the max() macro using statement-expressions, which can't be used
// outside a function. Define the simple version for use below
#define MY_MAX(a, b) ((a) > (b)) ? (a) : (b)

// Computed from the networking parameters and packet size
// See https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
const unsigned long PACKET_AIRTIME_MICROS = (unsigned long) (
  1000.0 * ((1 << LORA_SF) / RFM_BW) * (
    (LORA_PREAMBLE + 4.25) + // Preamble
    (8.0 + MY_MAX(
      0.0,
      ceil((8.0 * sizeof(packet::Packet) - 4.0 * LORA_SF + 44.0) / (4.0 * LORA_SF)) * LORA_CR
    )) // Payload
  ) + 0.5 // For rounding
);
const unsigned long TDMA_SLOT_SIZE = PACKET_AIRTIME_MICROS + TDMA_SLOT_PADDING;

// Instance of radio driver over SPI
RFM95 radio = new Module(RFM_CS_PIN, RFM_G0_PIN, RFM_RST_PIN);

// Buffer
packet::Packet packetBuffer;

// Interrupt handler states
volatile bool dataReceived = false;
volatile unsigned long receiveTime = 0UL;
volatile bool newCycleStarted = false;
volatile unsigned long cycleStartTime = 0UL;
volatile bool readyToTransmit = false;
volatile bool cycleComplete = false;

// Disable receive interrupt when it's not needed
volatile bool enableReceiveInterrupt = true;

// TDMA algorithm state
volatile int tdmaTotalSlots = 0;
volatile int tdmaMySlot = -1;
volatile bool tdmaStartupPhase = true;
int tdmaCycleNeighborCount = 0;
long tdmaCycleOffset = 0L;
bool tdmaTxComplete = false;
uint8_t tdmaPrevSlotUnusedCycles = 0;
uint8_t tdmaLastSlotUnusedCycles = 0;



void setup() {
  DEBUG_BEGIN(57600);

  pinMode(LED_PIN, OUTPUT);

  // Manually reset RFM95W
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  // Initialize radio
  // See https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
  int radioState = radio.begin(
    RFM_FREQ,
    RFM_BW,
    LORA_SF,
    LORA_CR,
    RADIOLIB_SX127X_SYNC_WORD,
    RFM_OUTPUT_POWER,
    LORA_PREAMBLE,
    0 // Automatic gain control
  );

  if (radioState != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not initialize RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  DEBUG_PRINT(F("RFM95 initialized, packet airtime "));
  DEBUG_PRINTLN(PACKET_AIRTIME_MICROS);

  // Set receive interrupt handler and start listening
  radio.setDio0Action(handleReceive);
  radioState = radio.startReceive();

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINTLN(F("Listening..."));
  }
  else {
    DEBUG_PRINT(F("Could not start receive mode on RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  tdmaStartup();
}



void loop() {
  if (newCycleStarted) {
    newCycleStarted = false;
    tdmaStartCycle();
  }

  if (readyToTransmit) {
    readyToTransmit = false;
    digitalWrite(LED_PIN, HIGH);
    transmit();
    tdmaEndSlot();
    radio.startReceive();
  }

  if (dataReceived) {
    enableReceiveInterrupt = false;
    dataReceived = false;

    receive();
    tdmaProcessPacket(packetBuffer);

    enableReceiveInterrupt = true;
    radio.startReceive();
  }

  if (cycleComplete) {
    cycleComplete = false;
    digitalWrite(LED_PIN, LOW);

    // Make sure we complete any radio operation that's currently ongoing
    waitForRadio();
    radio.sleep();
    tdmaEndCycle();
  }

  DEBUG_FLUSH();
  goToSleep();
}



void tdmaStartup() {
  // Initial startup phase timer
  clock::setTimeout(tdmaHandleStartupTimeout, TDMA_CYCLE_DURATION * 3);
}



void tdmaSetCycleStartTimeout() {
  // Adjust our cycle start time based on the average discrepancy between the expected start time
  // for each slot versus the actual receive time, including ourselves in the average by adding
  // one to the denominator (from our perspective, our offset of course is zero).
  long adjustment = tdmaCycleOffset / (tdmaCycleNeighborCount + 1);

  // Wake up a little early to make sure we hear (or properly schedule) the first slot.
  unsigned long cycleStartTimeout = TDMA_CYCLE_DURATION - TDMA_SLOT_SIZE + adjustment - (
    clock::micros() - cycleStartTime
  );

  clock::setTimeout(tdmaHandleCycleStartTimeout, cycleStartTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA sleep for "));
  DEBUG_PRINT(cycleStartTimeout);
  DEBUG_PRINT(F(" adjustment="));
  DEBUG_PRINTLN(adjustment);
}



void tdmaSetCycleEndTimeout() {
  // Listen for a couple extra slots in case someone new is trying to join
  unsigned long cycleEndTimeout = TDMA_SLOT_SIZE * (tdmaTotalSlots + 2) - (
    clock::micros() - cycleStartTime
  );
  clock::setTimeout(tdmaHandleCycleEndTimeout, cycleEndTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA cycle ends in "));
  DEBUG_PRINTLN(cycleEndTimeout);
}



void tdmaSetTransmitTimeout() {
  unsigned long transmitTimeout = TDMA_SLOT_SIZE * tdmaMySlot;
  unsigned long now = clock::micros();

  // We wake up a little before the first slot, so the cycleStartTime may be in the future.
  if ((long) (cycleStartTime - now) > 0) {
    transmitTimeout += cycleStartTime - now;
  }
  else {
    transmitTimeout -= now - cycleStartTime;
  }

  clock::setTimeout(tdmaHandleTransmitTimeout, transmitTimeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" Will transmit in "));
  DEBUG_PRINTLN(transmitTimeout);
}



void tdmaHandleStartupTimeout() {
  // All alone
  DEBUG_PRINT(clock::micros());
  DEBUG_PRINTLN(F(" TDMA alone"));

  tdmaMySlot = 0;
  tdmaTotalSlots = 1;
  tdmaStartupPhase = false;
  tdmaHandleCycleStartTimeout();
}



void tdmaHandleCycleStartTimeout() {
  // We always wake up one TDMA_SLOT_SIZE early
  cycleStartTime = clock::micros() + TDMA_SLOT_SIZE;
  newCycleStarted = true;
}



void tdmaHandleCycleEndTimeout() {
  cycleComplete = true;
}



void tdmaHandleTransmitTimeout() {
  readyToTransmit = true;
}



void tdmaStartCycle() {
  tdmaCycleNeighborCount = 0;
  tdmaCycleOffset = 0L;
  tdmaTxComplete = false;

  if (tdmaMySlot > 0) {
    tdmaPrevSlotUnusedCycles++;
  }

  if (tdmaMySlot != tdmaTotalSlots - 1) {
    tdmaLastSlotUnusedCycles++;
  }

  tdmaSetTransmitTimeout();

  if (tdmaMySlot != 0) {
    radio.startReceive();
  }

  DEBUG_PRINT(F("<-> "));
  DEBUG_PRINTLN(cycleStartTime);
}



void tdmaEndSlot() {
  tdmaTxComplete = true;
  tdmaSetCycleEndTimeout();
}



void tdmaEndCycle() {
  if (tdmaStartupPhase) {
    // We heard a cycle during the startup phase! End the startup phase and pick a slot.
    tdmaStartupPhase = false;
    tdmaMySlot = tdmaTotalSlots;
    tdmaTotalSlots++;
    tdmaPrevSlotUnusedCycles = 0;
    tdmaLastSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA join | slot="));
    DEBUG_PRINT(tdmaMySlot);
    DEBUG_PRINT(F(" total="));
    DEBUG_PRINTLN(tdmaTotalSlots);
  }
  else if (tdmaMySlot > 0 && tdmaPrevSlotUnusedCycles >= TDMA_STEAL_AFTER_UNUSED_CYCLES) {
    // The slot below us isn't in use, steal it to defragment the cycle. If we were in the last
    // slot, reduce our total slots (everyone else will figure it out eventually).
    if (tdmaMySlot == tdmaTotalSlots - 1) {
      tdmaTotalSlots--;
    }
    tdmaMySlot--;
    tdmaPrevSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA my slot now "));
    DEBUG_PRINTLN(tdmaMySlot);
  }

  if (
    tdmaMySlot < tdmaTotalSlots - 1 &&
    tdmaLastSlotUnusedCycles >= TDMA_STEAL_AFTER_UNUSED_CYCLES
  ) {
    // The last slot is unused, reduce our total slots to reduce the overall cycle time.
    tdmaTotalSlots--;
    tdmaLastSlotUnusedCycles = 0;

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA total slots decreased to "));
    DEBUG_PRINTLN(tdmaTotalSlots);
  }

  tdmaSetCycleStartTimeout();
}



void tdmaProcessPacket(const packet::Packet& packet) {
  // TODO: do we need to worry about transmission delay?
  unsigned long neighborCycleStartTime = (
    receiveTime -
    packet.delayMillis * 1000UL -
    PACKET_AIRTIME_MICROS -
    packet.slot * TDMA_SLOT_SIZE
  );

  if (tdmaStartupPhase && cycleStartTime == 0UL) {
    // This is the first neighbor we heard in the startup phase! We don't have a cycleStartTime
    // yet, so make a first approximation here. We'll make a finer adjustment at the end of the
    // cycle, just like in all future cycles.
    cycleStartTime = neighborCycleStartTime;
  }

  // Track the discrepancy between the expected start time for each slot versus the actual
  // receive time. This will be used to keep us in sync with our neighbors.
  tdmaCycleNeighborCount++;
  tdmaCycleOffset += (long) (neighborCycleStartTime - cycleStartTime);

  if (packet.slot == tdmaMySlot - 1) {
    // Track when the previous slot is unusued so we can steal it
    tdmaPrevSlotUnusedCycles = 0;
  }
  else if (packet.slot == tdmaMySlot) {
    // Uh oh, we have a conflict!
    // If we haven't already transmitted, cancel transmission and pick a new slot when the cycle
    // completes -- essentially leaving and re-joining the network.
    if (!tdmaTxComplete) {
      tdmaStartupPhase = true;
      tdmaSetCycleEndTimeout();
    }
  }
  else if (packet.slot == tdmaTotalSlots - 1) {
    // Track when the last slot is unused so we can shrink the cycle accordingly
    tdmaLastSlotUnusedCycles = 0;
  }
  else if (packet.slot >= tdmaTotalSlots) {
    // A new neighbor! Update our state and reset our end of cycle timer if necesssary.
    tdmaTotalSlots = packet.slot + 1;
    tdmaLastSlotUnusedCycles = 0;

    if (tdmaStartupPhase || tdmaTxComplete) {
      // We already transmitted or are still starting up, so reschedule our end-of-cycle sleep.
      // (This should always be the case unless the new neighbor transmitted way too early.)
      tdmaSetCycleEndTimeout();
    }

    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" TDMA total slots increased to "));
    DEBUG_PRINTLN(tdmaTotalSlots);
  }
}



void handleReceive() {
  if (enableReceiveInterrupt) {
    receiveTime = clock::micros();
    dataReceived = true;
  }
}



void debugPrintPacketBuffer() {
#if DEBUG
  for (unsigned int i = 0; i < sizeof(packet::Packet) / sizeof(uint8_t); i++) {
    DEBUG_PRINT(F(" 0x"));
    DEBUG_PRINT(((uint8_t*) &packetBuffer)[i], HEX);
  }
  DEBUG_PRINTLN();
#endif
}



void goToSleep() {
  LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
}



void waitForRadio() {
  // Only bits 0, 1, and 3 are meaningful according to the datasheet, if they're all clear, the
  // radio isn't in use.
  while ((radio.getModemStatus() & 0x0b)) {
    yield();
  }
}



void receive() {
  int radioState = radio.readData((uint8_t*) &packetBuffer, sizeof(packet::Packet));

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(receiveTime);
    DEBUG_PRINT(F(" RECEIVE | expected="));
    DEBUG_PRINT(cycleStartTime + packetBuffer.slot * TDMA_SLOT_SIZE);
    DEBUG_PRINT(F(" actual="));
    DEBUG_PRINT(receiveTime - packetBuffer.delayMillis * 1000UL);
    DEBUG_PRINT(F(" data:"));
    debugPrintPacketBuffer();
  }
  else {
    DEBUG_PRINT(F("Could not receive, code "));
    DEBUG_PRINTLN(radioState);
  }
}



void transmit() {
  waitForRadio();

  unsigned long delayMillis = (
    clock::micros() - tdmaMySlot * TDMA_SLOT_SIZE - cycleStartTime + 500UL /* for rounding */
  ) / 1000UL;

  if (delayMillis >= 1 << sizeof(packetBuffer.delayMillis)) {
    // Too long a delay to put in the packet, we've certainly missed our slot anyway, just skip
    // this transmission
    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" Skipping tx delayMillis="));
    DEBUG_PRINTLN(delayMillis);
    return;
  }

  packetBuffer.slot = tdmaMySlot;
  packetBuffer.delayMillis = delayMillis;
  // These two fields are not currently in use
  packetBuffer.flags = 0;
  packetBuffer.reserved = 0;

  // Send data, disabling interrupt while tranmitting (otherwise it's triggered on TX done)
  bool receiveInterruptPrevState = enableReceiveInterrupt;

  enableReceiveInterrupt = false;
  int radioState = radio.transmit((uint8_t*) &packetBuffer, sizeof(packet::Packet));
  enableReceiveInterrupt = receiveInterruptPrevState;

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" SENT"));
    debugPrintPacketBuffer();
  }
  else {
    DEBUG_PRINT(F("Could not transmit, code "));
    DEBUG_PRINTLN(radioState);
  }
}
