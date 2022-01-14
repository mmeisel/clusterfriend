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
// nodes announce the total number of slots in use, as well as their slot number (e.g. 3 of 9).
// A new node takes the maximum slot count it hears and adds itself at the end. So if it heard that
// there are 9 slots from its neighbors, it adds a tenth slot, announcing itself as 10 of 10.
// On each cycle, nodes update their total slot count based on the *maximum* number of slots they
// hear from any of their neighbors.
//
// If a node hears no other nodes in its initial listening period, it starts announcing itself as
// 1 of 1. To avoid collisions between two nodes that boot at the same time, 1 of 1 announcements
// occur only ever other cycle, and non-deterministically switch phase.
//
// TODO: figure out how to defragment the slots as nodes leave.
//
#define CYCLE_DURATION 5000000UL // 5 seconds in microseconds
#define SLOT_PADDING 30000UL // How much silence to allow in each slot, 30 ms in microseconds

// Pins
#define RFM_RST_PIN 2
#define RFM_G0_PIN 3
#define RFM_CS_PIN 4
#define RFM_G1_PIN 5
#define LED_PIN 8


// Computed from the networking parameters and packet size
// See https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
const unsigned long PACKET_AIRTIME_MICROS = (unsigned long) (
  1000.0 * ((1 << LORA_SF) / RFM_BW) * (
    (LORA_PREAMBLE + 4.25) + // Preamble
    (8.0 + max(
      0.0,
      ceil((8.0 * sizeof(packet::Packet) - 4.0 * LORA_SF + 44.0) / (4.0 * LORA_SF)) * LORA_CR)
    ) // Payload
  ) + 0.5 // For rounding
);
const unsigned long SLOT_SIZE = PACKET_AIRTIME_MICROS + SLOT_PADDING;

// Instance of radio driver over SPI
RFM95 radio = new Module(RFM_CS_PIN, RFM_G0_PIN, RFM_RST_PIN, RFM_G1_PIN);

// Buffer
packet::Packet packet_buffer;

// Interrupt handler states
volatile bool data_received = false;
volatile unsigned long receive_time = 0UL;
volatile bool new_cycle_started = false;
volatile unsigned long cycle_start_time = 0UL;
volatile bool ready_to_transmit = false;
volatile bool cycle_complete = false;

// Disable receive interrupt when it's not needed
volatile bool enable_receive_interrupt = true;

// TDMA algorithm state
volatile int tdma_total_slots = 0;
volatile int tdma_my_slot = -1;
volatile bool tdma_startup_phase = true;
int tdma_cycle_neighbor_count = 0;
long tdma_cycle_offset = 0L;



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
  int radio_state = radio.begin(
    RFM_FREQ,
    RFM_BW,
    LORA_SF,
    LORA_CR,
    RADIOLIB_SX127X_SYNC_WORD,
    RFM_OUTPUT_POWER,
    LORA_PREAMBLE,
    0 // Automatic gain control
  );

  if (radio_state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not initialize RFM95, code "));
    DEBUG_PRINTLN(radio_state);
    while (true);
  }

  DEBUG_PRINT(F("RFM95 initialized, packet airtime "));
  DEBUG_PRINTLN(PACKET_AIRTIME_MICROS);

  // Set receive interrupt handler and start listening
  radio.setDio0Action(handle_receive);
  radio_state = radio.startReceive();

  if (radio_state == RADIOLIB_ERR_NONE) {
    DEBUG_PRINTLN(F("Listening..."));
  }
  else {
    DEBUG_PRINT(F("Could not start receive mode on RFM95, code "));
    DEBUG_PRINTLN(radio_state);
    while (true);
  }

  // Initial startup phase timer
  clock::set_timeout(handle_startup_timeout, CYCLE_DURATION * 3);
}



void loop() {
  if (new_cycle_started) {
    new_cycle_started = false;
    tdma_cycle_neighbor_count = 0;
    tdma_cycle_offset = 0L;

    set_transmit_timeout();

    if (tdma_my_slot != 0) {
      radio.startReceive();
    }

    DEBUG_PRINT(F("<-> "));
    DEBUG_PRINTLN(cycle_start_time);
  }

  if (ready_to_transmit) {
    ready_to_transmit = false;
    // TODO: phase transmission if I'm alone
    digitalWrite(LED_PIN, HIGH);
    transmit();
    set_cycle_end_timeout();
    radio.startReceive();
  }

  if (data_received) {
    enable_receive_interrupt = false;

    receive();

    if (tdma_startup_phase && cycle_start_time == 0UL) {
      // This is the first neighbor we heard in the startup phase! We don't have a cycle_start_time
      // yet, so make a first approximation here. We'll make a finer adjustment at the end of the
      // cycle, just like in all future cycles.
      cycle_start_time = (
        receive_time -
        packet_buffer.delay_millis * 1000UL -
        PACKET_AIRTIME_MICROS -
        packet_buffer.slot * SLOT_SIZE
      );
    }

    // Track the discrepancy between the expected start time for each slot versus the actual
    // receive time. This will be used to keep us in sync with our neighbors.
    tdma_cycle_neighbor_count++;
    // TODO: do we need to worry about transmission delay?
    tdma_cycle_offset += (
      (long) (receive_time - cycle_start_time) -
      packet_buffer.delay_millis * 1000L -
      PACKET_AIRTIME_MICROS -
      packet_buffer.slot * SLOT_SIZE
    );

    if (packet_buffer.total_slots > tdma_total_slots) {
      // More neighbors than we thought! Update our state and reset our end of cycle timer
      // if necessary.
      tdma_total_slots = packet_buffer.total_slots;

      if (packet_buffer.slot > tdma_my_slot) {
        // We already transmitted (or are still starting up), so reschedule our end-of-cycle sleep
        set_cycle_end_timeout();
      }
    }

    data_received = false;
    enable_receive_interrupt = true;
    radio.startReceive();
  }

  if (cycle_complete) {
    cycle_complete = false;
    digitalWrite(LED_PIN, LOW);

    radio.sleep();

    if (tdma_startup_phase) {
      // We heard a cycle during the startup phase! End the startup phase and pick a slot.
      tdma_startup_phase = false;
      tdma_my_slot = tdma_total_slots;
      tdma_total_slots++;

      DEBUG_PRINT(clock::micros());
      DEBUG_PRINT(F(" TDMA join | slot="));
      DEBUG_PRINT(tdma_my_slot);
      DEBUG_PRINT(F(" total="));
      DEBUG_PRINTLN(tdma_total_slots);
    }

    set_cycle_start_timeout();
  }

  DEBUG_FLUSH();
  sleep();
}



void set_cycle_start_timeout() {
  // Adjust our cycle start time based on the average discrepancy between the expected start time
  // for each slot versus the actual receive time, including ourselves in the average by adding
  // one to the denominator (from our perspective, our offset of course is zero).
  long adjustment = tdma_cycle_offset / (tdma_cycle_neighbor_count + 1);

  // Wake up a little early to make sure we hear (or properly schedule) the first slot.
  unsigned long cycle_start_timeout = CYCLE_DURATION - SLOT_SIZE + adjustment - (
    clock::micros() - cycle_start_time
  );

  // TODO: sanity check on timeout value?

  clock::set_timeout(handle_cycle_start_timeout, cycle_start_timeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA sleep for "));
  DEBUG_PRINT(cycle_start_timeout);
  DEBUG_PRINT(F(" adjustment="));
  DEBUG_PRINTLN(adjustment);
}



void set_cycle_end_timeout() {
  unsigned long cycle_end_timeout = SLOT_SIZE * (tdma_total_slots + 2) - (
    clock::micros() - cycle_start_time
  );
  clock::set_timeout(handle_cycle_end_timeout, cycle_end_timeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" TDMA cycle ends in "));
  DEBUG_PRINTLN(cycle_end_timeout);
}



void set_transmit_timeout() {
  unsigned long transmit_timeout = SLOT_SIZE * tdma_my_slot;
  unsigned long now = clock::micros();

  // We wake up a little before the first slot, so the cycle_start_time may be in the future.
  if ((long) (cycle_start_time - now) > 0) {
    transmit_timeout += cycle_start_time - now;
  }
  else {
    transmit_timeout -= now - cycle_start_time;
  }

  clock::set_timeout(handle_transmit_timeout, transmit_timeout);

  DEBUG_PRINT(clock::micros());
  DEBUG_PRINT(F(" Will transmit in "));
  DEBUG_PRINTLN(transmit_timeout);
}



void handle_startup_timeout() {
  // All alone
  DEBUG_PRINT(clock::micros());
  DEBUG_PRINTLN(F(" TDMA alone"));

  tdma_my_slot = 0;
  tdma_total_slots = 1;
  tdma_startup_phase = false;
  handle_cycle_start_timeout();
}



void handle_cycle_start_timeout() {
  // We always wake up one SLOT_SIZE early
  cycle_start_time = clock::micros() + SLOT_SIZE;
  new_cycle_started = true;
}



void handle_cycle_end_timeout() {
  cycle_complete = true;
}



void handle_transmit_timeout() {
  ready_to_transmit = true;
}



void handle_receive() {
  if (enable_receive_interrupt) {
    // TODO: this is the receive *complete* interrupt, I really need the preamble one...
    receive_time = clock::micros();
    data_received = true;
  }
}



void debug_print_packet_buffer() {
#if DEBUG
  for (unsigned int i = 0; i < sizeof(packet::Packet) / sizeof(uint8_t); i++) {
    DEBUG_PRINT(F(" 0x"));
    DEBUG_PRINT(((uint8_t*) &packet_buffer)[i], HEX);
  }
  DEBUG_PRINTLN();
#endif
}



void sleep() {
  LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
}



void receive() {
  int radio_state = radio.readData((uint8_t*) &packet_buffer, sizeof(packet::Packet));

  if (radio_state == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(receive_time);
    DEBUG_PRINT(F(" RECEIVE | expected="));
    DEBUG_PRINT(cycle_start_time + packet_buffer.slot * SLOT_SIZE);
    DEBUG_PRINT(F(" actual="));
    DEBUG_PRINT(receive_time - packet_buffer.delay_millis * 1000UL);
    DEBUG_PRINT(F(" data:"));
    debug_print_packet_buffer();
  }
  else {
    DEBUG_PRINT(F("Could not receive, code "));
    DEBUG_PRINTLN(radio_state);
  }
}



void transmit() {
  packet_buffer.slot = tdma_my_slot;
  packet_buffer.total_slots = tdma_total_slots;
  packet_buffer.delay_millis = (uint16_t) (
    (clock::micros() - tdma_my_slot * SLOT_SIZE - cycle_start_time + 500UL /* for rounding */)
    / 1000UL
  );

  // Send data, disabling interrupt while tranmitting (otherwise it's triggered on TX done)
  bool receive_interrupt_prev_state = enable_receive_interrupt;

  enable_receive_interrupt = false;
  int radio_state = radio.transmit((uint8_t*) &packet_buffer, sizeof(packet::Packet));
  enable_receive_interrupt = receive_interrupt_prev_state;

  if (radio_state == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" SENT"));
    debug_print_packet_buffer();
  }
  else {
    DEBUG_PRINT(F("Could not transmit, code "));
    DEBUG_PRINTLN(radio_state);
  }
}
