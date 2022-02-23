#include <Arduino.h>
#include <nrf24l01.h>

Nrf24l01 MyNrf(8, 7);
uint8_t nrf_addr[5] = {0x34, 0x43, 0x10, 0x10, 0x01};

uint8_t rx_buf[32] = {0};
uint8_t rx_len = 0;
uint8_t value;

void setup()
{
    Serial.begin(115200);
    Serial.print("Hello tx demo!\n");

    MyNrf.init();
    if (!MyNrf.is_available())
    {
        Serial.print("Device not available!\n");
    }
    MyNrf.length_mode = Nrf24l01::LENGTH_DYN;
    MyNrf.txrx_mode = Nrf24l01::MODE_TX;
    MyNrf.channel = 40;
    MyNrf.config();
}

void loop()
{
    value = (uint8_t) random(255);
    Serial.print("sending ");
    Serial.println(value);
    MyNrf.set_tx_addr(nrf_addr, sizeof(nrf_addr));
    MyNrf.tx_packet((uint8_t *)&value, sizeof(value));
    delay(1000);
}
