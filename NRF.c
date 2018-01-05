#include <msp430.h>
#include <stdint.h>
#include "nRF24L01.h"
#include "NRF.h"
#include "main.h"




//write a byte return a byte (at the same time)
//sleep while waiting for the process
uint8_t NRF_RW(uint8_t write_byte)
{
    UCA0TXBUF = write_byte;
    __bis_SR_register(LPM0_bits + GIE);   //sleep, wait for rx int
    //__delay_cycles(100); //timing adjust, need to delay csn, WARNING if you mess with clocks, gotta change this!!!
    return (get_rx_buffer());  //return the read byte
}

//send a raw command
uint8_t NRF_cmd(uint8_t cmd)
{
    uint8_t status;
    NRF_set_csn(0);                 // Set CSN low, init SPI tranaction
    status = NRF_RW(cmd);
    NRF_set_csn(1);                 // Set CSN high again
    return (status);
}

uint8_t NRF_write(uint8_t reg, uint8_t data)
{
	uint8_t status;
	NRF_set_csn(0);        //set csn low
	status = NRF_RW(reg + W_REGISTER);
	NRF_RW(data);
	NRF_set_csn(1);    //set csn high
	return (status); 
}

uint8_t NRF_read(uint8_t reg)
{
    uint8_t read_byte;
    NRF_set_csn(0);                 // Set CSN low, init SPI tranaction
    NRF_RW(reg + R_REGISTER);
	read_byte = NRF_RW(0); // Perform SPI_RW to read unsigned char from nRF24L01
	NRF_set_csn(1);                 // Set CSN high again
    return (read_byte);
}

uint8_t NRF_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status, i;
	NRF_set_csn(0);        //set csn low
	status = NRF_RW(reg);
	for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
	{
		NRF_RW(*pBuf++);
	}
	NRF_set_csn(1);     //set csn high
	return (status); 
}

//read a register, return the resulting data byte(s) into pBuf. need to provide the number of bytes to read back for a given register
uint8_t NRF_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status, i;
    NRF_set_csn(0);                  // Set CSN low, init SPI tranaction
    status = NRF_RW(reg);
    for (i = 0; i < bytes; i++)
    {
        pBuf[i] = NRF_RW(0); // Perform SPI_RW to read unsigned char from nRF24L01
    }
    NRF_set_csn(1);                 // Set CSN high again
    return (status);                  // return nRF24L01 status unsigned char
}

uint8_t NRF_check_chip(){
    uint8_t setup_aw = NRF_read(SETUP_AW);
    if(setup_aw >= 1 && setup_aw <= 3){
        return (1);
    }
    return (0);
}


uint8_t NRF_read_status(){
    return(NRF_read(NRF_STATUS));

    //below doesnt work. dont know why
    //supposed to be twice as fast as a regular reg read
    /*
    uint8_t status;
    NRF_set_csn(0);                 // Set CSN low, init SPI tranaction
    status = NRF_RW(NRF_STATUS + R_REGISTER);
    NRF_set_csn(1);                 // Set CSN high again
    return(status);
    */

}

void NRF_carrier_test_mode(){
    NRF_set_ce(0);
    NRF_write(NRF_CONFIG, (PWR_UP));
    NRF_write(RF_SETUP, (CONT_WAVE + PLL_LOCK + RF_PWR_LOW + RF_PWR_HIGH));
    NRF_write(RF_CH, 40);
    NRF_set_ce(1);
    uint8_t status = NRF_read(NRF_STATUS);
}

void NRF_power_mode(uint8_t i){
    //read modify write
    uint8_t read = NRF_read(NRF_CONFIG);
    if (i == 1)
        NRF_write(NRF_CONFIG, (read|PWR_UP));     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
    else
        NRF_write(NRF_CONFIG, (read&(~PWR_UP)) );     //enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
}

void NRF_clear_status(){
    NRF_write(NRF_STATUS, NRF_read_status());   //clear status flags
}


