/**
 * Radio module
 *
 * This module provides all the required functions to manage the nRF51822
 * transceiver.
 **/

#pragma once
#include <Arduino.h>

extern uint8_t *rx_buffer; /* Rx buffer used by RF to store packets. */

#define IRQ_PRIORITY_HIGHEST 0
#define IRQ_PRIORITY_HIGH 1
#define IRQ_PRIORITY_MEDIUM 2
#define IRQ_PRIORITY_LOW 3

#define RADIO_MODE_MODE_Ieee802154_250Kbit (15UL)


#define IEEE802154_FRAME_LEN_MAX (127U)   /**< maximum 802.15.4 frame length */
#define IEEE802154G_FRAME_LEN_MAX (2047U) /**< maximum 802.15.4g-2012 frame length */
#define IEEE802154_ACK_FRAME_LEN (5U)     /**< ACK frame length */
#define IEEE802154_FCS_LEN                  (2U)


#define RADIO_PCNF0_PLEN_32bitZero (2UL) /*!< 32-bit zero preamble - used for IEEE 802.15.4 */
#define RADIO_PCNF0_CRCINC_Include (1UL) /*!< LENGTH includes CRC */
#define RADIO_PCNF0_CRCINC_Pos (26UL) /*!< Position of CRCINC field. */
#define RADIO_CRCCNF_SKIPADDR_Ieee802154 (2UL) /*!< CRC calculation as per 802.15.4 standard. Starting at first byte after length field. */


#define RADIO_INTENSET_FRAMESTART_Pos (14UL) /*!< Position of FRAMESTART field. */
#define RADIO_INTENSET_FRAMESTART_Msk (0x1UL << RADIO_INTENSET_FRAMESTART_Pos) /*!< Bit mask of FRAMESTART field. */
#define RADIO_INTENSET_CCAIDLE_Pos (17UL) /*!< Position of CCAIDLE field. */
#define RADIO_INTENSET_CCAIDLE_Msk (0x1UL << RADIO_INTENSET_CCAIDLE_Pos) /*!< Bit mask of CCAIDLE field. */
#define RADIO_INTENSET_CCABUSY_Pos (18UL) /*!< Position of CCABUSY field. */
#define RADIO_INTENSET_CCABUSY_Msk (0x1UL << RADIO_INTENSET_CCABUSY_Pos) /*!< Bit mask of CCABUSY field. */


#define ED_RSSISCALE        (4U)    /**< RSSI scale for internal HW value */
#define ED_RSSIOFFS         (-92)   /**< RSSI offset for internal HW value */

/**
 * @brief IEEE802.15.4 default value for CCA threshold (in dBm)
 */
#ifndef CONFIG_IEEE802154_CCA_THRESH_DEFAULT
#define CONFIG_IEEE802154_CCA_THRESH_DEFAULT       (-70)
#endif

/* Bits 15..8 : CCA energy busy threshold. Used in all the CCA modes except CarrierMode. */
#define RADIO_CCACTRL_CCAEDTHRES_Pos (8UL) /*!< Position of CCAEDTHRES field. */
#define RADIO_CCACTRL_CCAEDTHRES_Msk (0xFFUL << RADIO_CCACTRL_CCAEDTHRES_Pos) /*!< Bit mask of CCAEDTHRES field. */

/**
 * @brief   Default start frame delimiter
 */
#define IEEE802154_SFD (0xa7)

uint8_t channel_to_freq(int channel);
void radio_disable(void);
void radio_set_sniff(int channel, uint32_t access_address);
void radio_send_custom(uint8_t *pBuffer, uint8_t channel);
void radio_tx_to_rx();