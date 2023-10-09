#include <Arduino.h>
#include "radio.h"
// #include "helpers.h"
#include <assert.h>
/**
 * channel_to_freq(int channel)
 *
 * Convert a BLE channel number into the corresponding frequency offset
 * for the nRF51822.
 **/

uint8_t channel_to_freq(int channel)
{
  assert(channel > 10);
  assert(channel < 27);
  return 5 * (channel - 10);
}

/**
 * radio_disable()
 *
 * Disable the radio.
 **/

void radio_disable(void)
{
  if (NRF_RADIO->STATE > 0)
  {
    NVIC_DisableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
      ;
  }
}

/**
 * @brief   Convert from dBm to the internal representation, when the
 *          radio operates as a IEEE802.15.4 transceiver.
 */
static inline uint8_t _dbm_to_ieee802154_hwval(int8_t dbm)
{
  return ((dbm - ED_RSSIOFFS) / ED_RSSISCALE);
}

static int set_cca_threshold(int8_t threshold)
{

  if (threshold < ED_RSSIOFFS)
  {
    return -1;
  }

  uint8_t hw_val = _dbm_to_ieee802154_hwval(threshold);

  // NRF_RADIO->CCACTRL &= ~RADIO_CCACTRL_CCAEDTHRES_Msk;
  *(uint32_t *)(0x4000166c) &= ~RADIO_CCACTRL_CCAEDTHRES_Msk;

  // NRF_RADIO->CCACTRL |= hw_val << RADIO_CCACTRL_CCAEDTHRES_Pos;
  *(uint32_t *)(0x4000166c) |= hw_val << RADIO_CCACTRL_CCAEDTHRES_Pos;
  return 0;
}

/**
 * radio_set_sniff(int channel)
 *
 * Configure the nRF51822 to sniff on a specific channel.
 **/

void radio_set_sniff(int channel, uint32_t access_address)
{
  // Disable radio
  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;
  NRF_RADIO->TXADDRESS = 0;

  /* Set 802154 data rate. */
  NRF_RADIO->MODE = RADIO_MODE_MODE_Ieee802154_250Kbit;
  /* Listen on channel . */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);

  /* set start frame delimiter */
  // NRF_RADIO->RESERVED12[4] = IEEE802154_SFD;
  *(uint32_t *)(0x40001660) = IEEE802154_SFD;

  /* set MHR filters */
  // NRF_RADIO->MHRMATCHCONF = 0;         /* Search Pattern Configuration */
  *(uint32_t *)(0x40001644) = 0;

  // NRF_RADIO->MHRMATCHMAS = 0xff0007ff; /* Pattern mask */
  *(uint32_t *)(0x40001648) = 0xff0007ff;

  /* and set some fitting configuration */
  NRF_RADIO->PCNF0 = ((8 << RADIO_PCNF0_LFLEN_Pos) |
                      (RADIO_PCNF0_PLEN_32bitZero << RADIO_PCNF0_PLEN_Pos) |
                      (RADIO_PCNF0_CRCINC_Include << RADIO_PCNF0_CRCINC_Pos));

  NRF_RADIO->PCNF1 = IEEE802154_FRAME_LEN_MAX;

  NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast; // Enable fast mode for radio ramp up

  // Enable CRC
  NRF_RADIO->CRCCNF = ((RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                       (RADIO_CRCCNF_SKIPADDR_Ieee802154 << RADIO_CRCCNF_SKIPADDR_Pos));
  NRF_RADIO->CRCPOLY = 0x11021;
  NRF_RADIO->CRCINIT = 0;

  // DATAWHITE
  NRF_RADIO->DATAWHITEIV = 0x40;

  NRF_RADIO->PACKETPTR = (uint32_t)(rx_buffer); 

  // configure CCA
  set_cca_threshold(CONFIG_IEEE802154_CCA_THRESH_DEFAULT);

  // Configure interrupts
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk |
                        RADIO_INTENSET_FRAMESTART_Msk |
                        // RADIO_INTENSET_CCAIDLE_Msk |
                        RADIO_INTENSET_CCABUSY_Msk;
  // Enable NVIC Interrupt for Radio
  NVIC_SetPriority(RADIO_IRQn, IRQ_PRIORITY_LOW);
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  // Enable receiver hardware
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_READY == 0)
    ;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
}

/**
 * Send raw data asynchronously.
 **/

void radio_send_custom(uint8_t *pBuffer, uint8_t channel)
{

  /* No shorts on disable. */
  NRF_RADIO->SHORTS = 0x0;
  /* Switch radio to TX. */
  radio_disable();

  /* Switch packet buffer to tx_buffer. */
  NRF_RADIO->PACKETPTR = (uint32_t)pBuffer;
  // NRF_RADIO->INTENSET = 1 << RADIO_INTENSET_END_Pos;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  Serial.write("While 1\r\n");
  while (!NRF_RADIO->EVENTS_READY)
    ;
  // NRF_RADIO->EVENTS_CCAIDLE = 0
  *(uint32_t *)(0x40001144) = 0;
  // NRF_RADIO->TASKS_CCASTART = 0
  *(uint32_t *)(0x4000102c) = 1;

  // cannnot pass this, consider not configure CCA mode so comment this line
  // while(!NRF_RADIO->EVENTS_CCAIDLE);
  Serial.write("While 2\r\n");
  while (!*(uint32_t *)(0x40001144))
    ;
  digitalWrite(PIN_LED2, HIGH);
  /* Transmit with max power. */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);

  // enable receiver
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_TXEN = 1;

  Serial.write("While 3\r\n");
  while (!NRF_RADIO->EVENTS_READY)
    ;
  NRF_RADIO->TASKS_START = 1;

  /* From now, radio will send data and notify the result to Radio_IRQHandler */
}

/**
 * Change radio from TX to RX, while keeping last configuration of radio_send_custom or radio_set_sniff
 **/
void radio_tx_to_rx()
{
  NRF_RADIO->PACKETPTR = (uint32_t)(rx_buffer);
  NVIC_DisableIRQ(RADIO_IRQn);
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

  // NRF_RADIO->INTENSET = 0;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->EVENTS_DISABLED == 0)
    ;

  // NRF_RADIO->EVENTS_READY = 0;
  // NRF_RADIO->TASKS_RXEN = 1;
}