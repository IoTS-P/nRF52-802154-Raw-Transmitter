#include <Arduino.h>
#include "radio.h"

// LED PIN after test 
#define PIN_LED1 2
#define PIN_LED2 3
#define PIN_LED3 4
#define PIN_LED4 5

// channel 
#define DEFAULT_INITIAL_CHANNEL 26

// read buffer  
uint8_t *rx_buffer;

// the frame length in physic layer header 
uint8_t frame_len = 0;
// the length without FCS
uint8_t pkt_len = 0;
// has data transmitting
bool transmitting = false;


// packtes
#define MAX_PKTS_IN 12 // Maximum packets in the queue
struct __attribute__((packed)) packets_structure
{
  uint16_t size[MAX_PKTS_IN] = {0};
  uint8_t *pkt_buffer[MAX_PKTS_IN];
  uint8_t head = 0;
  uint8_t tail = 0;
} packets;

// channel
uint8_t radio_channel = DEFAULT_INITIAL_CHANNEL;

void radio_802154_setup(int channel)
{
  radio_set_sniff(channel, 0);
}

/**
 * nRF51822 RADIO handler.
 *
 * This handler is called whenever a RADIO event occurs (IRQ).
 **/
extern "C" void RADIO_IRQHandler(void)
{
  // if (NRF_RADIO->EVENTS_FRAMESTART)
  if (*(uint32_t *)(0x40001138))
  {
    *(uint32_t *)(0x40001138) = 0;
  }
  if (NRF_RADIO->EVENTS_READY)
  {
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_START = 1;
  }

  if (NRF_RADIO->EVENTS_END)
  {
    NRF_RADIO->EVENTS_END = 0;
    // NRF_RADIO->TASKS_START = 1;
    // If transmitt end
    if (transmitting)
    {
      transmitting = false;
      Serial.write("SENDED ");
      Serial.print(packets.head);
      Serial.write("\r\n");
      digitalWrite(PIN_LED1, HIGH);
      // free unused buffer
      free(packets.pkt_buffer[packets.head]);
      packets.size[packets.head] = 0;
      // add the header pointer to set the packet is sent 
      packets.head = (packets.head + 1) % MAX_PKTS_IN;
      radio_tx_to_rx();
      return;
    }
  }
  if (*(uint32_t *)(0x40001144))
  {
    *(uint32_t *)(0x40001144) = 0;
    // NRF_RADIO->TASKS_CCASTART = 0
    *(uint32_t *)(0x4000102c) = 1;
  }
}

void setup()
{
  // put your setup code here, to run once:
  // buffer full signal
  pinMode(PIN_LED4, OUTPUT);
  digitalWrite(PIN_LED4, HIGH);
  // send packets signal
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, HIGH);
  // CCABUSY signal
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, HIGH);
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  rx_buffer = (uint8_t *)malloc(IEEE802154_FRAME_LEN_MAX + 3);
  radio_802154_setup(radio_channel);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (NRF_RADIO->EVENTS_READY)
  {
    NRF_RADIO->EVENTS_READY = 0;
    // NRF_RADIO->TASKS_START;
  }

  // memcpy pkt_buffer to tx_buffer
  if (!transmitting && packets.size[packets.head])
  {
    Serial.write("SEND BUFFER: len ");
    Serial.print(packets.size[packets.head]);
    Serial.write(" head: ");
    Serial.print(packets.head);
    Serial.write("\r\n value:");
    for (size_t i = 0; i < packets.size[packets.head]; i++)
    {
      Serial.write(" ");
      Serial.print(packets.pkt_buffer[packets.head][i], 16);
    }
    Serial.write("\r\n");
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED4, HIGH);
    transmitting = true;
    radio_send_custom(packets.pkt_buffer[packets.head], radio_channel);
  }

  // read buffer
  while (Serial.available())
  {
    frame_len = Serial.read();
    pkt_len = frame_len - IEEE802154_FCS_LEN;
    Serial.write("INPUT: ");
    Serial.print(pkt_len);
    Serial.write(" tail:");
    Serial.print(packets.tail);
    Serial.write("\r\n");
    uint8_t *work_buffer = (uint8_t *)malloc(pkt_len + 1);
    // add two byte for FCS 
    work_buffer[0] = frame_len;
    Serial.readBytes(work_buffer + 1, pkt_len);
    // add buffer to tail
    packets.size[packets.tail] = pkt_len + 1;
    packets.pkt_buffer[packets.tail] = work_buffer;
    packets.tail = (packets.tail + 1) % MAX_PKTS_IN;
    if (packets.head == packets.tail)
      digitalWrite(PIN_LED4, LOW);
  }
}
