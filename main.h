#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

void led (uint8_t i);
void act_on_command(uint8_t command);
uint8_t get_rx_buffer();
void NRF_set_csn(uint8_t bit);
void NRF_set_ce(uint8_t bit);

__interrupt void USCIA0RX_ISR(void);
__interrupt void Port_2(void);
#endif /* MAIN_H_ */
