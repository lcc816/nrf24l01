#include <Arduino.h>
#include <nrf24l01.h>

Nrf24l01 MyNrf(8, 7);
uint8_t nrf_addr[5] = {0x34, 0x43, 0x10, 0x10, 0x01};

uint8_t rx_buf[32] = {0};
uint8_t rx_len = 0;

void setup()
{
    Serial.begin(115200);
    Serial.print("Hello rx demo!\n");

    MyNrf.init();
    if (!MyNrf.is_available())
    {
        Serial.print("Device not available!\n");
    }
    MyNrf.set_rx_addr(1, nrf_addr, sizeof(nrf_addr));
    MyNrf.length_mode = RF24_LENGTH_DYN;
    MyNrf.txrx_mode = RF24_MODE_RX;
    MyNrf.channel = 40;
    MyNrf.config();

}

void loop()
{
    if (MyNrf.is_data_ready())
    {
        rx_len = MyNrf.get_data(rx_buf);
        if (rx_len > 0)
        {
            Serial.print("get value: ");
            for (size_t i = 0; i < rx_len; i++)
            {
                Serial.print(rx_buf[i]);
                Serial.print(' ');
            }
            Serial.print('\n');
        }
    }
}
