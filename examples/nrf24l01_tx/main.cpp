#include <Arduino.h>
#include <nrf24l01.h>

Nrf24l01 MyNrf(8, 7);
uint8_t address[][6] = {"1Node", "2Node"};
// which address this node will use
uint8_t node = 0;
uint8_t peer_node = 1;

uint8_t rx_buf[32] = {0};
uint8_t rx_len = 0;
float payload = 9.0;

void setup()
{
    Serial.begin(115200);
    while (!Serial); // wait to connect serial
    Serial.print("Hello nRF24L01 demo!\n");

    MyNrf.init();
    if (!MyNrf.is_available())
    {
        Serial.print("Device not available!\n");
        while (1) {} // hold in infinite loop
    }

    // user settings
    MyNrf.dynamic_length = false;
    MyNrf.payload_len = sizeof(payload);
    MyNrf.data_rate = RF24_1MBPS;
    MyNrf.repeat_cnt = 15;
    MyNrf.txrx_mode = RF24_MODE_RX;
    MyNrf.channel = 76;
    MyNrf.power_level = RF24_PWR_LVL1;
    MyNrf.config();

    // To set the radioNumber via the Serial monitor on startup
    Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
    while (!Serial.available()) {
    // wait for user input
    }
    char input = (char)Serial.parseInt();
    node = (input == 1);
    peer_node = (node == 0);
    Serial.print("Selected node number:");
    Serial.println((int)node);

    MyNrf.set_tx_addr(address[peer_node], 5);
    MyNrf.set_rx_addr(1, address[node], 5);

    Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
}

void loop()
{
    if (MyNrf.txrx_mode == RF24_MODE_TX)
    {
        unsigned long start_timer = micros();
        int8_t report = MyNrf.tx_packet((uint8_t *)&payload, sizeof(payload));
        unsigned long end_timer = micros();
        if (report)
        {
            Serial.print(F("Transmission failed with reason: "));
            Serial.println((int)report);
        }
        else
        {
            Serial.print(F("Transmission successful! "));          // payload was delivered
            Serial.print(F("Time to transmit = "));
            Serial.print(end_timer - start_timer);                 // print the timer result
            Serial.print(F(" us. Sent: "));
            Serial.println(payload);                               // print payload sent
            payload += 0.01;
        }
        delay(1000);
    }
    else
    {
        if (MyNrf.is_data_ready())
        {
            rx_len = MyNrf.get_data(rx_buf);
            if (rx_len > 0)
            {
                Serial.print("Received ");
                Serial.print(rx_len);
                Serial.print(" bytes: ");
                payload = *(float *)rx_buf;
                Serial.println(payload);
            }
        }
    }

    if (Serial.available()) {
        // change the role via the serial monitor

        char c = toupper(Serial.read());
        if (c == 'T' && (MyNrf.txrx_mode != RF24_MODE_TX)) {
            // Become the TX node

            MyNrf.txrx_mode = RF24_MODE_TX;
            Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
            MyNrf.set_txrx_mode(RF24_MODE_TX);

        } else if (c == 'R' && (MyNrf.txrx_mode != RF24_MODE_RX)) {
            // Become the RX node

            MyNrf.txrx_mode = RF24_MODE_RX;
            Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
            MyNrf.set_txrx_mode(RF24_MODE_RX);
        }
    }
}
