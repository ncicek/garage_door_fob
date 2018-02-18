#ifndef RADIO_FUNC_H_
#define RADIO_FUNC_H_

void Common_NRF_Config(void);
void TX_Mode(void);
void RX_Mode(void);
void listen();
void transmit();
void NRF_set_csn(uint8_t bit);
void NRF_set_ce(uint8_t bit);

#endif /* RADIO_FUNC_H_ */
