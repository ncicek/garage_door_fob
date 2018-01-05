/*
 * NRF.h
 *
 *  Created on: Dec 29, 2017
 *      Author: Nebi
 */

#ifndef NRF_H_
#define NRF_H_
#include <stdint.h>
uint8_t NRF_cmd(uint8_t cmd);
uint8_t NRF_write(uint8_t reg, uint8_t data);
uint8_t NRF_read(uint8_t reg);
uint8_t NRF_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
uint8_t NRF_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
uint8_t NRF_RW(uint8_t write_byte);

uint8_t NRF_check_chip();
uint8_t NRF_read_status();
void NRF_carrier_test_mode();
void NRF_power_mode(uint8_t i);
void NRF_clear_status();

#endif /* NRF_H_ */
