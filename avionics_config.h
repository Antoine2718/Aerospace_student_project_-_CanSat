// ------------------------ CONFIG ------------------------
#define USE_LORA 1            // 1 = LoRa (SX127x), 0 = RFM69
#define LOG_BUFFER_SIZE 512   // ATTENTION RAM (2KB total)
#define LOG_FLUSH_INTERVAL_MS 2000UL  // flush SD toutes les 2s si données
#define TICK_MS 10            // tick period (cooperative scheduler)
#define GPS_BAUD 9600
#define SERIAL_BAUD 115200

// Hardware pins
const uint8_t PIN_LED = 13;
const uint8_t PIN_SD_CS = 10;
const uint8_t PIN_LORA_CS = 8;
const uint8_t PIN_LORA_RST = 9;
const uint8_t PIN_IMU_INT = 2;  // interruption from IMU (if used)
const uint8_t PIN_BATT_DIV = A0; // voltage divider measurement
const float BATT_DIVIDER_RATIO = (100.0 + 100.0) / 100.0; // exemple 100k/100k
const float ADC_REF = 3.3; // si Pro Mini 3.3V

// EEPROM addresses for snapshot
const int EEPROM_ADDR_SNAPSHOT = 0;
