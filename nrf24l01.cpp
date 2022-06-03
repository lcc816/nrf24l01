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
    _ce_pin = ce_pin;
    _csn_pin = csn_pin;
    spi = spi_obj;
    txrx_mode = RF24_MODE_RX;
    channel = 1;
    dynamic_length = false;
    payload_len = 32;
    repeat_cnt = 10;
    data_rate = RF24_1MBPS;
    power_level = RF24_PWR_MAX;
}

void Nrf24l01::init()
{
    pinMode(_ce_pin, OUTPUT);
    pinMode(_csn_pin, OUTPUT);

    ce_low();
    csn_high();
    //spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    spi->begin();
}

void Nrf24l01::config()
{
    delay(5);
    if (dynamic_length)
    {
        // Enable dynamic payload length data pipe 0
        spi_write_reg(REG_DYNPD, MASK_DPL_P0);
        // Enables Dynamic Payload Length, Payload with ACK,
        // and the W_TX_PAYLOAD_NOACK command
        spi_write_reg(REG_FEATRUE, MASK_EN_DPL | MASK_EN_ACK_PAY | MASK_EN_DYN_ACK);
    }
    else
    {
        payload_len = min(32, payload_len);
        set_payload_size(payload_len);
        // Disable dynamic payloads for all pipes
        spi_write_reg(REG_DYNPD, 0);
        // Note that Payload with ACK is not supported in Dynamic Payload Length mode,
        // clear the FEATURE
        spi_write_reg(REG_FEATRUE, 0);
    }

    // enable auto-ack on all pipes
    spi_write_reg(REG_EN_AA, 0x3F);
    // Only enable RX pipe 0 & 1
    spi_write_reg(REG_EN_RXADDR, MASK_ERX_P0 | MASK_ERX_P1);
    spi_write_reg(REG_SETUP_AW, AW_5BYTES);
    spi_write_reg(REG_SETUP_RETR, ARD_1500US |
                  (repeat_cnt & MASK_ARC));
    spi_write_reg(REG_RF_CH, channel);

    // RF setup
    set_data_rate(data_rate);
    set_power(power_level);

    _clear_status_flags();
    // Flush buffers
    flush_rx();
    flush_tx();

    spi_write_reg(REG_CONFIG, MASK_EN_CRC | MASK_CRCO);
    power_up();
    // Set to a certain mode
    set_txrx_mode(txrx_mode);
}

// Only RX_ADDR_P0 and RX_ADDR_P1 have a width of 5 bytes
// RX_ADDR_P2~5 only have LSB
// and the MSBytes of RX_ADDR_P2~5 are equal to RX_ADDR_P1[39:8]
void Nrf24l01::set_rx_addr(uint8_t pipe, uint8_t *addr, uint8_t len)
{
    uint8_t value;
    len = (len > 5) ? 5 : len;
    pipe = (pipe > 5) ? 5 : pipe;

    spi_write_buffer(REG_RX_ADDR_P0 + pipe, addr, 5);
    // Enable data pipe
    value = spi_read_reg(REG_EN_RXADDR);
    spi_write_reg(REG_EN_RXADDR, value | (1 << pipe));
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

void Nrf24l01::set_payload_size(uint8_t size)
{
    if (size > 32)
        size = 32;

    for (uint8_t i = 0; i < 6; i++)
        spi_write_reg(REG_RX_PW_P0 + i, size);
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
    digitalWrite(_ce_pin, HIGH);
}

void Nrf24l01::ce_low()
{
    digitalWrite(_ce_pin, LOW);
}

void Nrf24l01::csn_high()
{
    digitalWrite(_csn_pin, HIGH);
}

void Nrf24l01::csn_low()
{
    digitalWrite(_csn_pin, LOW);
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

void Nrf24l01::set_data_rate(rf24_datarate_e data_rate)
{
    uint8_t setup = spi_read_reg(REG_RF_SETUP);
    setup &= ~(MASK_RF_DR_HIGH | MASK_RF_DR_LOW);
    setup |= _parse_data_rate_reg(data_rate);

    spi_write_reg(REG_RF_SETUP, setup);
}

void Nrf24l01::spi_write_reg(uint8_t addr, uint8_t data)
{
    // New cmd must be started by a high to low transition on CSN
    csn_low();
    _status = spi->transfer(W_REGISTER | addr);
    spi->transfer(data);
    csn_high();
}

uint8_t Nrf24l01::spi_read_reg(uint8_t addr)
{
    uint8_t ret = 0x0;

    csn_low();
    _status = spi->transfer(R_REGISTER | addr);
    ret = spi->transfer(0xFF);
    csn_high();

    return ret;
}

void Nrf24l01::spi_write_buffer(uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    csn_low();
    _status = spi->transfer(W_REGISTER | addr);
    for (int i = 0; i < bytes; i++)
    {
        spi->transfer(buffer[i]);
    }
    csn_high();
}

void Nrf24l01::spi_read_buffer(uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    csn_low();
    _status = spi->transfer(R_REGISTER | addr);
    for (int i = 0; i < bytes; i++)
    {
        buffer[i] = spi->transfer(0xFF);
    }
    csn_high();
}

uint8_t Nrf24l01::_parse_data_rate_reg(rf24_datarate_e data_rate)
{
    // Encoding:
    // [RF_DR_LOW, RF_DR_HIGH]:
    // ‘00’ – 1Mbps
    // ‘01’ – 2Mbps
    // ‘10’ – 250kbps
    // ‘11’ – Reserved
    if (data_rate == RF24_250KBPS)
        return MASK_RF_DR_LOW;
    else if (data_rate == RF24_2MBPS)
        return MASK_RF_DR_HIGH;
    else
        return 0;
}

uint8_t Nrf24l01::_parse_power_reg(rf24_power_e level, bool extension)
{
    /*
     * | level (enum value) | nRF24L01 | Si24R1 with extension bit = 1 | Si24R1 with extension bit = 0 |
     * |:------------------:|:-------:|:--------:|:-------:|
     * |   RF24_PWR_LVL0    | -18 dBm |  -6 dBm  | -12 dBm |
     * |   RF24_PWR_LVL1    | -12 dBm |  -0 dBm  | -4 dBm  |
     * |   RF24_PWR_LVL2    | -6 dBm  |  3 dBm   | 1 dBm   |
     * |   RF24_PWR_LVL3    |  0 dBm  |  7 dBm   | 4 dBm   |
     */
    if (level > RF24_PWR_MAX)
        level = RF24_PWR_MAX;
    return (level << 1) + extension;
}

void Nrf24l01::_clear_status_flags()
{
    // Clear the flags in STATUS register
    spi_write_reg(REG_STATUS, MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT);
}

void Nrf24l01::set_power(rf24_power_e level, bool extension)
{
    uint8_t setup = spi_read_reg(REG_RF_SETUP);
    setup &= ~(MASK_RF_PWR | MASK_RF_OBS);
    setup |= _parse_power_reg(level, extension);

    spi_write_reg(REG_RF_SETUP, setup);
}

void Nrf24l01::set_txrx_mode(rf24_mode_e mode)
{
    uint8_t config_reg = spi_read_reg(REG_CONFIG);

    if (mode == RF24_MODE_RX)
    {
        power_up();
        config_reg |= MASK_PRIM_RX;
        spi_write_reg(REG_CONFIG, config_reg);
        _clear_status_flags();
        ce_high();
    }
    else
    {
        ce_low();
        delayMicroseconds(100);
        config_reg &= ~MASK_PRIM_RX;
        spi_write_reg(REG_CONFIG, config_reg);
        // enable rx pip0 for auto ack
        spi_write_reg(REG_EN_RXADDR, spi_read_reg(REG_EN_RXADDR) | MASK_ERX_P0);
    }
}

void Nrf24l01::set_channel(uint8_t ch)
{
    ch = min(125, ch);
    spi_write_reg(REG_RF_CH, ch);
}

uint8_t Nrf24l01::get_channel()
{
    return spi_read_reg(REG_RF_CH);
}

void Nrf24l01::power_down()
{
    ce_low();
    spi_write_reg(REG_CONFIG, spi_read_reg(REG_CONFIG) & ~MASK_PWR_UP);
}

void Nrf24l01::power_up()
{
    uint8_t value = spi_read_reg(REG_CONFIG);
    if (!(value & MASK_PWR_UP))
    {
        value |= MASK_PWR_UP;
        spi_write_reg(REG_CONFIG, value);
        delayMicroseconds(5000);
    }
}

int8_t Nrf24l01::tx_packet(uint8_t *tx_buf, uint8_t len)
{
    uint8_t *ptr = tx_buf;
    uint8_t status;
    uint8_t timeout;

    uint8_t padding_len = 0;

    if (len > 32)
        len = 32;
    if (!dynamic_length)
    {
        if (len > payload_len)
            len = payload_len;
        padding_len = payload_len - len;
    }

    flush_tx();
    _clear_status_flags();
    // go to standby-I mode
    ce_low();
    // set to tx mode
    // set_txrx_mode(RF24_MODE_TX);
    csn_low();
    // Write cmd to write payload
    spi->transfer(W_TX_PAYLOAD);
    // Write payload
    while (len--)
    {
        spi->transfer(*ptr++);
    }
    while (padding_len--)
    {
        spi->transfer(0x0);
    }
    csn_high();

    // Start transmission
    ce_high();

    timeout = 30;
    do
    {
        delayMicroseconds(100);
        status = spi_read_reg(REG_STATUS);
        if (status & MASK_MAX_RT)
        {
            flush_tx();
            Serial.print(timeout); Serial.print("-"); Serial.print(_status); Serial.print("-"); Serial.println(status);
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
    if (spi_read_reg(REG_STATUS) & MASK_RX_DR)
        return true;

    if (spi_read_reg(REG_FIFO_STATUS) & MASK_RX_EMPTY)
        return false;

    return true;
}

uint8_t Nrf24l01::data_len()
{
    if (dynamic_length)
    {
        uint8_t len;
        csn_low();
        spi->transfer(R_RX_PL_WID);
        len = spi->transfer(0xFF);
        csn_high();
        return len;
    }
    else
    {
        return payload_len;
    }
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

void Nrf24l01::dump_reg()
{
    for (uint8_t i = 0; i <= 0x1D; i++)
    {
        uint8_t value = spi_read_reg(i);
        Serial.print(i, HEX); Serial.print(": ");
        Serial.println(value, HEX);
    }
}