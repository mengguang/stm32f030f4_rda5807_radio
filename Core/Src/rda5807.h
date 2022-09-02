#ifndef __RDA5807_H__
#define __RDA5807_H__

#include <stdio.h>
#include <stdbool.h>

//Random Access Address
#define RDA5807_ADDRESS 0x22
#define RDA5807_ID 0x58


typedef  uint16_t u16;
typedef  uint8_t u8;

void rda5807_init();
void rda5807_set_frequency(u16 new_frequency);
//void rda5807_print_rds(void);
void rda5807_set_mute(bool value);
bool rda5807_toggle_mute();
void rda5807_set_bass_boost(bool value);
bool rda5807_toggle_bass_boost();
void rda5807_set_volume(u8 new_volume);

void delay(uint32_t ms);

#endif // __RDA5807_H__
