#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

//database for storing previous_lfsrs for reciever, needs 256 unused bytes after this address

void led (uint8_t i);
void act_on_command(uint8_t command);
uint8_t get_rx_buffer();
void NRF_set_csn(uint8_t bit);
void NRF_set_ce(uint8_t bit);

void write_to_flash(uint16_t *dbl_byte_array, uint8_t len);


__interrupt void USCIA0RX_ISR(void);
__interrupt void Port_2(void);
#endif /* MAIN_H_ */
