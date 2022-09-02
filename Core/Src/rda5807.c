#include "rda5807.h"
#include "main.h"
#include "i2c.h"
#include <string.h>

static u8 tx[] = {0x02,
        0xF4, 0x01, // 0x02h
        0x00, 0x00,
        0x00, 0x00,
        0x88, 0x0F, //0x05h
        0x00, 0x00,
        0x7C, 0x12};

static u8 rx[8];

void zero_rx() {
	memset(rx,0,sizeof(rx));
}

void delay(uint32_t ms) {
	HAL_Delay(ms);
}

void I2C_Master_BufferWrite(uint8_t *data, uint8_t size) {
	HAL_I2C_Master_Transmit(&hi2c1, RDA5807_ADDRESS, data, size, 1000);
}

void I2C_Master_BufferRead(uint8_t *data, uint8_t size) {
	HAL_I2C_Master_Receive(&hi2c1, RDA5807_ADDRESS, data, size, 1000);
}

void rda5807_init() {
    tx[0] = 0x00;

    I2C_Master_BufferWrite(tx, 1);
    zero_rx();
    I2C_Master_BufferRead(rx, 2);

    if (rx[0] == RDA5807_ID) {
//        printf("Chip is present\n\r");
//    	while(true) {
//    		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    		HAL_Delay(1000);
//    	}
    }

    // Reset radio
    tx[0] = 0x02;
    tx[1] = 0x00;
    tx[2] = 0x03;
    I2C_Master_BufferWrite(tx, 3);
    delay(500);

    // Enable
    tx[1] = 0xC0;
    tx[2] = 0x0D;
    I2C_Master_BufferWrite(tx, 3);
    delay(500);

//    printf("Configuration done\r\n");
}

void rda5807_set_frequency(u16 new_frequency) {
    u16 freq = new_frequency;
    u16 freqB = freq - 870;
    u8 freqH = freqB >> 2;
    u8 freqL = (freqB & 3) << 6; // Shift channel selection for matching register 0x03

    tx[0] = 0x03;
    tx[1] = freqH;
    tx[2] = freqL + 0x10;
    I2C_Master_BufferWrite(tx, 3);
}

bool rda5807_toggle_mute() {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(tx, 1);
    I2C_Master_BufferRead(tx+1, 2);

    // Toggle mute bit
    tx[1] = tx[1] ^ 1 << 6;
    I2C_Master_BufferWrite( tx, 3);

    // Return new mute value
    return !(tx[1] & 1 << 6);
}

void rda5807_set_mute(bool value) {
    value = !value;

    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(tx, 1);
    I2C_Master_BufferRead(tx+1, 2);

    // Set mute bit
    tx[1] ^= (-value ^ tx[1]) & (1 << 6);
    I2C_Master_BufferWrite(tx, 3);
}

bool rda5807_toggle_bass_boost() {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(tx, 1);
    I2C_Master_BufferRead(tx+1, 2);

    // Toggle bass bit
    tx[1] = tx[1] ^ 1 << 4;
    I2C_Master_BufferWrite(tx, 3);

    // Return new bass value
    return (tx[1] & 1 << 4);

}

void rda5807_set_bass_boost(bool value) {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(tx, 1);
    I2C_Master_BufferRead(tx+1, 2);

    // Set mute bit
    tx[1] ^= (-value ^ tx[1]) & (1 << 4);
    I2C_Master_BufferWrite(tx, 3);
}

void rda5807_set_volume(u8 new_volume) {
    // Read current data
    tx[0] = 0x05;
    I2C_Master_BufferWrite(tx, 1);
    I2C_Master_BufferRead(tx+1, 2);

    tx[2] = (tx[2] & 0xF0) | new_volume;

    I2C_Master_BufferWrite(tx, 3);
}
