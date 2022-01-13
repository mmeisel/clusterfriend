// Time synchronization is based on the DESYNC-TDMA protocol
// (https://dash.harvard.edu/bitstream/handle/1/25680331/tr-18-06.pdf)
//
// Required libraries:
// - Low-Power v1.81.0 (https://github.com/rocketscream/Low-Power)
// - RadioLib v5.1.0 (https://github.com/jgromes/RadioLib)
#include <LowPower.h>
#include <RadioLib.h>

#include "debug.h"
#include "clock.h"



// Networking parameters
// Transmit/received packet size (used for buffers)
#define PACKET_SIZE 2
// Frequency for RFM95W
#define RFM_FREQ 915.0
// 2..20 dBm, 13 dBm is default
#define RFM_TX_POWER 13

// DESYNC-TDMA algorithm parameters
#define CYCLE_DURATION 1000000UL // 1 second in microseconds
#define DESYNC_ALPHA 0.95

// Other parameters
#define LED_ON_DURATION 200000UL // 200ms in microseconds

// Pins
#define RFM_RST_PIN 2
#define RFM_INT_PIN 3
#define RFM_CS_PIN 4
#define LED_PIN 8



// Instance of radio driver over SPI
RFM95 radio = new Module(RFM_CS_PIN, RFM_INT_PIN, RFM_RST_PIN);

// Buffer
uint8_t packet_buffer[PACKET_SIZE];

// Flag to indicate that a packet was received and the time of reception
volatile bool data_received = false;
volatile unsigned long receive_offset = 0UL;
volatile bool timer_fired = false;
volatile unsigned long my_last_offset = 0UL;

// Disable interrupt when it's not needed
volatile bool enable_receive_interrupt = true;

// DESYNC-TDMA algorithm state
bool waiting_for_next = false;
unsigned long next_offset = 0UL;
unsigned long prev_offset = 0UL;



void setup() {
#if DEBUG
  Serial.begin(9600);
#endif

  pinMode(LED_PIN, OUTPUT);

  // Manually reset RFM95W
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  // Initialize radio
  int radio_state = radio.begin();

  if (radio_state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not initialize RFM95, code "));
    DEBUG_PRINTLN(radio_state);
    while (true);
  }

  DEBUG_PRINTLN(F("RFM95 initialized"));

  // Set RFM95W frequency
  radio_state = radio.setFrequency(RFM_FREQ);

  if (radio_state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not set frequency on RFM95, code "));
    DEBUG_PRINTLN(radio_state);
    while (true);
  }

  DEBUG_PRINT(F("RFM95 frequency set to "));
  DEBUG_PRINT(RFM_FREQ);
  DEBUG_PRINTLN(F(" MHz"));

  // Set RFM95W transmit power from PA_BOOST pin
  radio.setOutputPower(RFM_TX_POWER, false);

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

  // Initialize DESYNC-TDMA timer
  clock::set_timeout(handle_timer, CYCLE_DURATION);
}



void loop() {
  if (timer_fired) {
    transmit();

    waiting_for_next = true;
    prev_offset = receive_offset;
    timer_fired = false;

    unsigned long next_timeout = CYCLE_DURATION - (clock::elapsed() - my_last_offset);
    clock::set_timeout(handle_timer, next_timeout);

    radio.startReceive();

    DEBUG_PRINT(F("@"));
    DEBUG_PRINT(my_last_offset);
    DEBUG_PRINT(F(" timeout="));
    DEBUG_PRINTLN(next_timeout);
  }

  if (data_received) {
    enable_receive_interrupt = false;

    int radio_state = radio.readData(packet_buffer, PACKET_SIZE);

    if (radio_state == RADIOLIB_ERR_NONE) {
      DEBUG_PRINT(F("Receiving @"));
      DEBUG_PRINT(receive_offset);
      debug_print_packet_buffer();
    }
    else {
      DEBUG_PRINT(F("Could not receive, code "));
      DEBUG_PRINTLN(radio_state);
    }

    // If the prev_offset is zero, we haven't received a previous message yet, since 0 is when
    // we fire. There would be a collision if another device fired at the same time.
    if (waiting_for_next && prev_offset > 0UL) {
      // This is a message from the neighbor that fires immediately after us, adjust our timer
      waiting_for_next = false;
      next_offset = receive_offset;

      unsigned long goal_offset = (unsigned long) (
        DESYNC_ALPHA * ((double) prev_offset + (double) next_offset) / 2.0 +
        0.5 // For rounding
      );
      unsigned long next_timeout = CYCLE_DURATION + goal_offset - clock::elapsed();

      clock::set_timeout(handle_timer, next_timeout);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));

      DEBUG_PRINT(F("@"));
      DEBUG_PRINT(receive_offset);
      DEBUG_PRINT(F(" next="));
      DEBUG_PRINT(next_offset);
      DEBUG_PRINT(F(" my="));
      DEBUG_PRINT(my_last_offset);
      DEBUG_PRINT(F(" prev="));
      DEBUG_PRINT(prev_offset);
      DEBUG_PRINT(F(" timeout="));
      DEBUG_PRINTLN(next_timeout);
    }

    data_received = false;
    enable_receive_interrupt = true;
    radio.startReceive();
  }

  DEBUG_FLUSH();
  sleep();
}



void handle_timer() {
  my_last_offset = clock::elapsed();
  timer_fired = true;
}



void handle_receive() {
  if (enable_receive_interrupt) {
    receive_offset = clock::elapsed();
    data_received = true;
  }
}



void debug_print_packet_buffer() {
#if DEBUG
  for (int i = 0; i < PACKET_SIZE; i++) {
    DEBUG_PRINT(F(" 0x"));
    DEBUG_PRINT(packet_buffer[i], HEX);
  }
  DEBUG_PRINTLN();
#endif
}



void sleep() {
  // radio.sleep();
  LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
}



void transmit() {
  packet_buffer[0] = 0xbe;
  packet_buffer[1] = 0xef;

  DEBUG_PRINT(F("Sending"));
  debug_print_packet_buffer();

  // Send data, disabling interrupt while tranmitting (otherwise it's triggered on TX done)
  enable_receive_interrupt = false;
  int radio_state = radio.transmit(packet_buffer, PACKET_SIZE);
  enable_receive_interrupt = true;

  if (radio_state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not transmit, code "));
    DEBUG_PRINTLN(radio_state);
  }
}
