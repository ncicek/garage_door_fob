#include <msp430.h>
#include "NRF.h"
#include <stdint.h>
#include "nRF24L01.h"
#include "radio_functions.h"
#include "main.h"

#include "crypto.h"

#define PLOAD_WIDTH 16  //16 bytes to transmit
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define SWITCH_TIME 65000
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x12,0x40,0xFE,0x17,0xC5};

//declare extern flags
extern volatile uint8_t timer_armed;
extern volatile uint8_t button_pressed;

void transmit(){
    static uint8_t previous_transmission_failed = 1; //track previous transmission status
    uint8_t n, i;
    if (previous_transmission_failed == 1)
        n = 2;  //double send if we have lost sync due to missed trasmission
    else
        n = 1;
    for (i=0; i<n; i++){    //repeat transmission n times
        uint8_t code[PLOAD_WIDTH];
        generate_code(TOGGLE_DOOR, code, SWITCH_TIME);

        NRF_power_mode(1); //power up
        NRF_cmd(FLUSH_TX);
        NRF_clear_status(); //clears all flags such as irq bit
        NRF_write_buf(W_TX_PAYLOAD, code, PLOAD_WIDTH);  //write the code as payload into the nrf24
        NRF_set_ce(1);  //transmit

        __bis_SR_register(LPM4_bits + GIE);  //sleep wake up on irq
        NRF_set_ce(0);  //stop transmitting
        NRF_power_mode(0); //power down radio

        uint8_t lost_packets = NRF_read(OBSERVE_TX)&0xf;
        uint8_t status = NRF_read_status();

        if (status&TX_DS){
            blink_led();
            previous_transmission_failed = 0;
        }
        else{
            previous_transmission_failed = 1;
        }
    }
}

void listen(){
    uint8_t recieved_code[PLOAD_WIDTH];
    uint8_t recieved_command;
    uint16_t time;
    NRF_power_mode(1); //power up
    NRF_set_ce(1); //must be high to listen

    //now sleep and wait for irq indicating recived packet
    while(1){
      while (timer_armed == 1){
          __bis_SR_register(LPM3_bits + GIE); // sleep with smclk on for timer to expire
      }
      NRF_cmd(FLUSH_RX);
      NRF_clear_status(); //clears all flags after each wakeup such as irq bit
      P1IE |= BIT3; //enable button int
      _bis_SR_register(LPM4_bits + GIE); //sleep wake up on irq
      uint8_t status = NRF_read_status();
      if (status & RX_DR) {

          NRF_read_buf(R_RX_PAYLOAD, recieved_code, PLOAD_WIDTH);    //load the recieved payload into code
          blink_led();

          if (decode_code(&recieved_command, recieved_code, &time) == 1){ //call decode_code on the recieved code to see if it is valid
              act_on_command(recieved_command, time);
          }
      }
      if (button_pressed == 1){ //the button in RX mode just checks SPI comms as a sanity check
          button_pressed = 0;
          if (NRF_check_chip()){
              blink_led();
          }
      }
    }
}

void TX_Mode(void)	//configure into tx mode
{
    Common_NRF_Config();
    NRF_write_buf(TX_ADDR + W_REGISTER, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
	NRF_write(NRF_CONFIG, (CRCO + EN_CRC ));
	NRF_clear_status();
}

void RX_Mode(void){
    Common_NRF_Config();
    NRF_write(RX_PW_P0, PLOAD_WIDTH); //reciever shall expect this payload lenght
    NRF_write(NRF_CONFIG, (CRCO + EN_CRC + PRIM_RX));
    NRF_clear_status();
}

void Common_NRF_Config(void){
    NRF_set_ce(0);         //set ce low

    NRF_write_buf(RX_ADDR_P0 + W_REGISTER, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
    NRF_write(EN_AA, ENAA_P0);      // Enable Auto.Ack:Pipe0
    NRF_write(EN_RXADDR, ERX_P0);  // Enable Pipe0
    NRF_write(SETUP_RETR, (ARD*15+ARC*15)); // 500us + 86us, 10 retransmit
    NRF_write(RF_CH, 0); //rf = 2400+0 =2.4GHZ avoid 80211wifi spectrum
    NRF_write(RF_SETUP, (RF_PWR_LOW + RF_PWR_HIGH + RF_DR_LOW));   // TX_PWR:0dBm, Datarate:250kbps (0x26)

}
