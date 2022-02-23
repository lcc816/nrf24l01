/*******************************************************************************
* @file     nrf24l01.cpp
* @author   lcc
* @version  1.0
* @date     10-Jan-2022
* @brief    Implementation of the NRF24L01 Class
*******************************************************************************/

#include "nrf24l01.h"

Nrf24l01::Nrf24l01(uint8_t ce_pin, uint8_t csn_pin, SPIClass *spi_obj)
{
    ce_pin_ = ce_pin;
    csn_pin_ = csn_pin;
    spi = spi_obj;
    txrx_mode = MODE_TX;
    channel = 1;
    length_mode = LENGTH_DYN;
    payload_len = 16;
    repeat_cnt = 10;
}

void Nrf24l01::init()
{
    pinMode(ce_pin_, OUTPUT);
    pinMode(csn_pin_, OUTPUT);

    ce_low();
    csn_high();
    spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    spi->begin();
}

void Nrf24l01::config()
{
    if (length_mode == LENGTH_DYN)
    {
        // Enable dynamic payload length data pipe 0
        spi_write_reg(REG_DYNPD, MASK_DPL_P0);
        // Enables Dynamic Payload Length, Payload with ACK,
        // and the W_TX_PAYLOAD_NOACK command
        spi_write_reg(REG_FEATRUE, MASK_EN_DPL | MASK_EN_ACK_PAY | MASK_EN_DYN_ACK);
    }
    else
    {
        spi_write_reg(REG_RX_PW_P0, payload_len);
        spi_write_reg(REG_RX_PW_P1, payload_len);
    }
    if (txrx_mode == MODE_RX)
    {
        spi_write_reg(REG_CONFIG, MASK_EN_CRC | MASK_PWR_UP | MASK_PRIM_RX);
    }
    else
    {
        spi_write_reg(REG_CONFIG, MASK_EN_CRC | MASK_PWR_UP);
    }
    spi_write_reg(REG_EN_AA, MASK_ENAA_P0);
    spi_write_reg(REG_EN_RXADDR, MASK_ERX_P0);
    spi_write_reg(REG_SETUP_AW, AW_5BYTES);
    spi_write_reg(REG_SETUP_RETR, ARD_4000US |
                  (repeat_cnt & MASK_ARC));
    spi_write_reg(REG_RF_CH, channel);
    // Set RF Data Rate to 250kbps
    // Set RF output power to 0dBm
    spi_write_reg(REG_RF_SETUP, MASK_RF_DR_LOW | MASK_RF_PWR);
}

// Only RX_ADDR_P0 and RX_ADDR_P1 have a width of 5 bytes
// RX_ADDR_P2~5 only have LSB
// and the MSBytes of RX_ADDR_P2~5 are equal to RX_ADDR_P1[39:8]
void Nrf24l01::set_rx_addr(uint8_t pipe, uint8_t *addr, uint8_t len)
{
    len = (len > 5) ? 5 : len;
    pipe = (pipe > 5) ? 5 : pipe;

    spi_write_buffer(REG_RX_ADDR_P0 + pipe, addr, 5);
}

void Nrf24l01::set_tx_addr(uint8_t *addr, uint8_t len)
{
    len = (len > 5) ? 5 : len;
    spi_write_buffer(REG_TX_ADDR, addr, len);
    /*
     * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
     */
    spi_write_buffer(REG_RX_ADDR_P0, addr, len);
}

bool Nrf24l01::is_available()
{
    uint8_t i;
    uint8_t tx_addr[5] = {'A', 'B', 'C', 'D', 'E'};
    uint8_t read_buf[5] = {0};
    spi_write_buffer(REG_TX_ADDR, tx_addr, 5);
    spi_read_buffer(REG_TX_ADDR, read_buf, 5);

    for (i = 0; i < 5; i++)
    {
        if (tx_addr[i] != read_buf[i])
        {
            return false;
        }
    }

    return true;
}

void Nrf24l01::ce_high()
{
    digitalWrite(ce_pin_, HIGH);
}

void Nrf24l01::ce_low()
{
    digitalWrite(ce_pin_, LOW);
}

void Nrf24l01::csn_high()
{
    digitalWrite(csn_pin_, HIGH);
}

void Nrf24l01::csn_low()
{
    digitalWrite(csn_pin_, LOW);
}

uint8_t Nrf24l01::flush_tx()
{
    uint8_t st = 0xFF;

    csn_low();
    st = spi->transfer(FLUSH_TX);
    csn_high();

    return st;
}

uint8_t Nrf24l01::flush_rx()
{
    uint8_t st = 0xFF;

    csn_low();
    st = spi->transfer(FLUSH_RX);
    csn_high();

    return st;
}

uint8_t Nrf24l01::spi_write_reg(uint8_t addr, uint8_t data)
{
    uint8_t st = 0xFF;

    // New cmd must be started by a high to low transition on CSN
    csn_low();
    st = spi->transfer(W_REGISTER | addr);
    spi->transfer(data);
    csn_high();

    return st;
}

uint8_t Nrf24l01::spi_read_reg(uint8_t addr, uint8_t *data)
{
    uint8_t st = 0xFF;

    csn_low();
    st = spi->transfer(R_REGISTER | addr);
    *data = spi->transfer(0xFF);
    csn_high();

    return st;
}

uint8_t Nrf24l01::spi_write_buffer(uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    uint8_t st = 0xFF;

    csn_low();
    st = spi->transfer(W_REGISTER | addr);
    for (int i = 0; i < bytes; i++)
    {
        spi->transfer(buffer[i]);
    }
    csn_high();

    return st;
}

uint8_t Nrf24l01::spi_read_buffer(uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    uint8_t st = 0xFF;

    csn_low();
    spi->transfer(R_REGISTER | addr);
    for (int i = 0; i < bytes; i++)
    {
        buffer[i] = spi->transfer(0xFF);
    }
    csn_high();

    return st;
}

void Nrf24l01::set_txrx_mode(ModeType mode)
{
    uint8_t control_reg = 0;
    spi_read_reg(REG_CONFIG, &control_reg);
    if (mode == MODE_TX)
    {
        control_reg |= MASK_PRIM_RX;
    }
    else
    {
        control_reg &= ~MASK_PRIM_RX;
    }
    spi_write_reg(REG_CONFIG, control_reg);
}

void Nrf24l01::set_channel(uint8_t ch)
{
    spi_write_reg(REG_RF_CH, ch);
}

uint8_t Nrf24l01::tx_packet(uint8_t *tx_buf, uint8_t len)
{
    uint8_t i;
    uint8_t status;
    uint8_t timeout;

    flush_tx();
    // go to standby-I mode
    ce_low();
    // set to tx mode
    set_txrx_mode(MODE_TX);
    csn_low();
    // Write cmd to write payload
    spi->transfer(W_TX_PAYLOAD);
    // Write payload
    for (i = 0; i < len; i++)
    {
        spi->transfer(tx_buf[i]);
    }
    csn_high();

    // Start transmission
    ce_high();

    timeout = 3;
    do
    {
        spi_read_reg(REG_STATUS, &status);
        if (status & MASK_MAX_RT)
        {
            flush_tx();
            return STATUS_MAX_RT;
        }
        if (status & MASK_TX_DS)
        {
            return STATUS_OK;
        }
    } while (timeout--);

    return STATUS_TIMEOUT;
}

// data is ready when: 1) RX_DR is asserted, 2) RX_DR is not asserted
// but RX FIFO is not empty.
bool Nrf24l01::is_data_ready()
{
    uint8_t value;

    spi_read_reg(REG_STATUS, &value);
    if (value & MASK_RX_DR)
        return true;
    spi_read_reg(REG_FIFO_STATUS, &value);
    if (value & MASK_RX_EMPTY)
        return false;
    return true;
}

uint8_t Nrf24l01::data_len()
{
    uint8_t len;
    csn_low();
    len = spi->transfer(R_RX_PL_WID);
    csn_high();
    return len;
}

// read data from the RX FIFO
uint8_t Nrf24l01::get_data(uint8_t *data)
{
    uint8_t len = data_len();
    // Flush RX FIFO if the read value is larger than 32 bytes
    if (len > 32)
    {
        len = 0;
        flush_rx();
        goto ret;
    }

    csn_low();
    spi->transfer(R_RX_PAYLOAD);
    for (int i = 0; i < len; i++)
        data[i] = spi->transfer(0xFF);
    csn_high();

ret:
    // Clear RX_DR flag
    spi_write_reg(REG_STATUS, MASK_RX_DR);
    return len;
}
