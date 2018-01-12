#include <msp430.h>
#include "NRF.h"
#include <stdint.h>
#include "nRF24L01.h"
#include "radio_functions.h"
#include "main.h"

#include "crypto.h"

#define PLOAD_WIDTH 16  //16 bytes to transmit
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x12,0x40,0xFE,0x17,0xC5};

extern volatile uint8_t button_pressed;
void transmit(){
    uint8_t code[PLOAD_WIDTH];
    generate_code(TOGGLE_DOOR, code);

    NRF_cmd(FLUSH_TX);
    NRF_clear_status(); //clears all flags such as irq bit
    NRF_write_buf(W_TX_PAYLOAD, code, PLOAD_WIDTH);  //write the code as payload into the nrf24
    NRF_power_mode(1); //power up
    NRF_set_ce(1);  //transmit
   
    __bis_SR_register(LPM4_bits + GIE);  //sleep wake up on irq
    NRF_set_ce(0);  //stop transmitting
    NRF_power_mode(0); //power down radio

    uint8_t lost_packets = NRF_read(OBSERVE_TX)&0xf;
    uint8_t status = NRF_read_status();

    if (status&TX_DS){
        blink_led();
    }
    else{

    }
}

void listen(){
    uint8_t recieved_code[PLOAD_WIDTH];
    uint8_t recieved_command;
    NRF_power_mode(1); //power up
    NRF_set_ce(1); //must be high to listen

    //now sleep and wait for irq indicating recived packet
    while(1){
        NRF_cmd(FLUSH_RX);
        NRF_clear_status(); //clears all flags after each wakeup such as irq bit
        P1IE |= BIT3; //enable button int
        _bis_SR_register(LPM4_bits + GIE); //sleep wake up on irq
        if (button_pressed & NRF_check_chip()){ //the button in RX mode just checks SPI comms as a sanity check
            blink_led();
            button_pressed = 0;
        }
        uint8_t status = NRF_read_status();
        if (status & RX_DR) {

            NRF_read_buf(R_RX_PAYLOAD, recieved_code, PLOAD_WIDTH);    //load the recieved payload into code
            blink_led();

            if (decode_code(&recieved_command, recieved_code) == 1){ //call decode_code on the recieved code to see if it is valid
                act_on_command(recieved_command);
            }
        }

    }
}

void TX_Mode(void)	//configure into tx mode
{
    Common_NRF_Config();
    NRF_write_buf(TX_ADDR + W_REGISTER, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    NRF_write(SETUP_RETR, (ARD*10+ARC*10)); // 500us + 86us, 10 retransmit
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
    NRF_write(RF_CH, 0); //rf = 2400+0 =2.4GHZ avoid 80211wifi spectrum
    NRF_write(RF_SETUP, (RF_PWR_LOW + RF_DR_LOW));   // TX_PWR:-12dBm, Datarate:250kbps (0x26)

}

void blink_led(){ //sets led high and sets timer to turn itoff after a while
    led(1);
    __delay_cycles(1000);
    led(0);
}
