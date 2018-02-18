#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#define TOGGLE_DOOR 1

//#define TX_MODE
#define RX_MODE

void led (uint8_t i);
void blink_led();
void act_on_command(uint8_t command, uint16_t time);
uint8_t get_rx_buffer();
inline void NRF_set_csn(uint8_t bit);
inline void NRF_set_ce(uint8_t bit);

void write_to_flash(uint16_t *dbl_byte_array, uint8_t len);
void init_flash_controller();
void read_from_flash(uint16_t *dbl_byte_array, uint8_t len);
void arm_timer1(uint16_t vlo_ticks);

inline void relay(uint8_t i);
inline void led(uint8_t i);

__interrupt void USCIA0RX_ISR(void);
__interrupt void Port_2(void);
#endif /* MAIN_H_ */
