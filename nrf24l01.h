/*******************************************************************************
* @file     nrf24l01.h
* @author   lcc
* @version  1.0
* @date     10-Jan-2022
* @brief    Register Map and methods of NRF24L01
*******************************************************************************/

#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include <Arduino.h>
#include <SPI.h>

/* Memory Map */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_RPD         0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATRUE     0x1D

/* Bit Mnemonics */
/* 00 - CONFIG */
#define MASK_RX_DR      ((uint8_t)0x40)
#define MASK_TX_DS      ((uint8_t)0x20)
#define MASK_MAX_RT     ((uint8_t)0x10)
#define MASK_EN_CRC     ((uint8_t)0x08)
#define MASK_CRCO       ((uint8_t)0x04)
#define MASK_PWR_UP     ((uint8_t)0x02)
#define MASK_PRIM_RX    ((uint8_t)0x01)
/* 01 - EN_AA */
#define MASK_ENAA_P5    ((uint8_t)0x20)
#define MASK_ENAA_P4    ((uint8_t)0x10)
#define MASK_ENAA_P3    ((uint8_t)0x08)
#define MASK_ENAA_P2    ((uint8_t)0x04)
#define MASK_ENAA_P1    ((uint8_t)0x02)
#define MASK_ENAA_P0    ((uint8_t)0x01)
/* 02 - EN_RXADDR */
#define MASK_ERX_P5     ((uint8_t)0x20)
#define MASK_ERX_P4     ((uint8_t)0x10)
#define MASK_ERX_P3     ((uint8_t)0x08)
#define MASK_ERX_P2     ((uint8_t)0x04)
#define MASK_ERX_P1     ((uint8_t)0x02)
#define MASK_ERX_P0     ((uint8_t)0x01)
/* 03 - SETUP_AW */
#define MASK_AW         ((uint8_t)0x03)
#define AW_ILLEGAL      ((uint8_t)0x0)
#define AW_3BYTES       ((uint8_t)0x1)
#define AW_4BYTES       ((uint8_t)0x2)
#define AW_5BYTES       ((uint8_t)0x3)
/* 04 - SETUP_RETR */
#define MASK_ARD        ((uint8_t)0xF0)
#define MASK_ARC        ((uint8_t)0x0F)
/* 05 - RF_CH */
#define MASK_RF_CH      ((uint8_t)0x7F)
/* 06 - RF_SETUP */
#define MASK_CONT_WAVE  ((uint8_t)0x80)
#define MASK_RF_DR_LOW  ((uint8_t)0x20)
#define MASK_PLL_LOCK   ((uint8_t)0x10)
#define MASK_RF_DR_HIGH ((uint8_t)0x08)
#define MASK_RF_PWR     ((uint8_t)0x06)
#define MASK_RF_OBS     ((uint8_t)0x01)
/* 07 - STATUS */
#define MASK_RX_DR      ((uint8_t)0x40)
#define MASK_TX_DS      ((uint8_t)0x20)
#define MASK_MAX_RT     ((uint8_t)0x10)
#define MASK_RX_P_NO    ((uint8_t)0x0E)
#define MASK_TX_FULL    ((uint8_t)0x01)
/* 08 - OBSERVE_TX */
#define MASK_PLOS_CNT   ((uint8_t)0xF0)
#define MASK_ARC_CNT    ((uint8_t)0x0F)
/* 09 - RPD */
#define MASK_RPD        ((uint8_t)0x01)
/* 17 - FIFO_STATUS */
#define MASK_TX_REUSE   ((uint8_t)0x40)
#define MASK_FIFO_FULL  ((uint8_t)0x20)
#define MASK_TX_EMPTY   ((uint8_t)0x10)
#define MASK_RX_FULL    ((uint8_t)0x02)
#define MASK_RX_EMPTY   ((uint8_t)0x01)
/* 1C - DYNPD */
#define MASK_DPL_P5     ((uint8_t)0x20)
#define MASK_DPL_P4     ((uint8_t)0x10)
#define MASK_DPL_P3     ((uint8_t)0x08)
#define MASK_DPL_P2     ((uint8_t)0x04)
#define MASK_DPL_P1     ((uint8_t)0x02)
#define MASK_DPL_P0     ((uint8_t)0x01)
/* 1D - FEATURE */
#define MASK_EN_DPL     ((uint8_t)0x04)
#define MASK_EN_ACK_PAY ((uint8_t)0x02)
#define MASK_EN_DYN_ACK ((uint8_t)0x01)

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define R_RX_PL_WID   0x60
#define W_ACK_PAYLOAD 0xA0
#define W_TX_PAYLOAD_NOACK  0xB0
#define NOP           0xFF

/* Status */
#define STATUS_OK       0
#define STATUS_FAIL     -1
#define STATUS_MAX_RT   -2
#define STATUS_TIMEOUT  -3

typedef enum
{
    RF24_MODE_TX = 0,
    RF24_MODE_RX
} rf24_mode_e;

typedef enum {
    /** (0) represents 1 Mbps */
    RF24_1MBPS = 0,
    /** (1) represents 2 Mbps */
    RF24_2MBPS,
    /** (2) represents 250 kbps */
    RF24_250KBPS
} rf24_datarate_e;

typedef enum {
    RF24_PWR_LVL0 = 0,
    RF24_PWR_LVL1,
    RF24_PWR_LVL2,
    RF24_PWR_LVL3,
    RF24_PWR_MAX = RF24_PWR_LVL3
} rf24_power_e;

class Nrf24l01
{
public:
    Nrf24l01(uint8_t ce_pin = 8, uint8_t csn_pin = 7,
             SPIClass *spi_obj = &SPI);

    void init();
    void config();
    bool is_available();

    void ce_low();
    void ce_high();

    void csn_low();
    void csn_high();

    void set_txrx_mode(rf24_mode_e mode);
    void set_channel(uint8_t ch);
    uint8_t get_channel();
    void set_retries(uint16_t delay, uint8_t count);
    void set_tx_addr(const uint8_t *addr, uint8_t len);
    void set_rx_addr(uint8_t pipe, const uint8_t *addr, uint8_t len);
    void set_payload_size(uint8_t size);
    uint8_t flush_tx();
    uint8_t flush_rx();

    void set_data_rate(rf24_datarate_e data_rate);
    /* Compatible with Si24R1, whose RF_PWR field has 3 bits */
    void set_power(rf24_power_e level, bool extension = true);

    int8_t tx_packet(uint8_t *tx_buf, uint8_t len);
    bool is_data_ready();
    uint8_t data_len();
    uint8_t get_data(uint8_t *data);

    void power_down();
    void power_up();

    // for debug
    void dump_reg();

    /* for transceiver setting */
    /* Tx or Rx */
    rf24_mode_e txrx_mode;
    /* Channel 0 - 127 */
    uint8_t channel;
    /* Dynamic Payload Lengt or Fixed Length */
    bool dynamic_length;
    /* The length of the payload can be from 0 to 32 bytes */
    uint8_t payload_len;
    /* Auto Retransmit Delay */
    uint16_t retry_delay;
    /* Auto Retransmit Count */
    uint8_t repeat_cnt;
    /* Data Rate */
    rf24_datarate_e data_rate;
    /* RF Power */
    rf24_power_e power_level;

private:
    /* CE Pin */
    uint8_t _ce_pin;
    /* CSN */
    uint8_t _csn_pin;
    /* SPI obj */
    SPIClass *spi;
    /* STATUS is shifted serially out on the MISO pin */
    uint8_t _status;

    void spi_write_reg(uint8_t addr, uint8_t data);
    uint8_t spi_read_reg(uint8_t addr);
    void spi_write_buffer(uint8_t addr, const uint8_t *buffer, uint8_t bytes);
    void spi_read_buffer(uint8_t addr, uint8_t *buffer, uint8_t bytes);
    uint8_t _parse_data_rate_reg(rf24_datarate_e data_rate);
    uint8_t _parse_power_reg(rf24_power_e level, bool extension);
    void _clear_status_flags();
};
#endif /* _NRF24L01_H_ */
