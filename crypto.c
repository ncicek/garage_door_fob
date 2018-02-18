//contains the lfsr and encrypt/decrypt routines

#include <msp430.h>
#include "TI_aes_128.h"
#include "main.h"
#include <stdint.h>
#include "crypto.h"
#include "secret_key.h"
#define polynomial 0xd008   //maximal lenght

uint16_t previous_lfsrs[32];
uint16_t lfsr; //current lfsr


//provide command to encrypt and get back a code
//generated_code must be uint8_t array with 16 elements
void generate_code(uint8_t command, uint8_t *generated_code, uint16_t time){
    #define bits_to_zero_pad (16-6) //modify as you more bytes of the generated_code

    lfsr = next_lfsr(lfsr);

	//construct the packet
	generated_code[0] = (uint8_t)(lfsr >> 8); //store msb
	generated_code[1] = (uint8_t)(lfsr & 0xFF); //store lsb
	generated_code[2] = command;
	generated_code[3] = get_chips_id();
	generated_code[4] = (uint8_t)(time >> 8);
    generated_code[5] = (uint8_t)(time & 0xFF);
	
	uint8_t loop;
	for (loop = (16-bits_to_zero_pad); loop < 16; loop++)	//zero pad
		generated_code[loop] = 0;	
	aes_enc_dec(generated_code,key,0);	//encrypt the constructed packet
	
}

//give a code and get back a command if its good
//return 1 if authenticated and valid code was found
uint8_t decode_code(uint8_t *command, uint8_t *generated_code, uint16_t *time){

	//static uint16_t previous_lfsr = initial_value;
	aes_enc_dec(generated_code,key,1);	//decrypt the packet

	uint8_t device_id = generated_code[3]; //pull out device id

	*time = (uint16_t)((generated_code[4] << 8) + generated_code[5]);

	//sanity check first
	if (generated_code[15] != 0)
	    return (0);
	if (device_id >= 32)
	    return (0);

	uint16_t previous_lfsr = get_previous_lfsr(device_id); //call function to get the previous lfsr of this particular device
	uint16_t current_lfsr = (generated_code[0]<<8) + generated_code[1];
    save_current_lfsr(current_lfsr, device_id); //store for the next time

	if (next_lfsr(previous_lfsr) == current_lfsr){	//if our next lfsr code matches the recieved lfsr code
		if (generated_code[15] == 0){	//if 16th byte is a zero
			*command = generated_code[2];	//then we are good
			return(1);
		}			
	}
	

	command = 0;
	return(0);	
}

uint16_t next_lfsr(uint16_t lfsr){
	uint8_t lsb = lfsr & 1;  /* Get lsb (i.e., the output bit). */
	lfsr >>= 1;               /* Shift register */
	if (lsb == 1)             /* Only apply toggle mask if output bit is 1. */
		lfsr ^= polynomial;        /* Apply toggle mask, value has 1 at bits corresponding to taps, 0 elsewhere*/

	return(lfsr);
}

//return a "unique" identifier of the chip
uint8_t get_chips_id(){
    return(1);

}

void generate_seed(){
    uint8_t i;
    ADC10CTL0 = ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
    ADC10CTL1 = INCH_1;                       // input A1
    ADC10AE0 |= 0x02;                         // PA.1 ADC option select

    for (i=0; i < 15; i++){
        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
        __bis_SR_register(CPUOFF + GIE);
        lfsr += ADC10MEM; //accumulate adc readings
    }

    //turn off ADC
    ADC10CTL0 = 0;
    ADC10CTL1 = 0;
    ADC10AE0 = 0;
}


// pass it a device id and it will return that devices' previous_lfsr
//if device is unknown or something else, it will return garbage
uint16_t get_previous_lfsr(uint8_t device_id){
    return(previous_lfsrs[device_id]);
}

//write lfsr to ram and save ram to flash
void save_current_lfsr(uint16_t current_lfsr, uint8_t device_id){
    previous_lfsrs[device_id] = current_lfsr;
    write_to_flash(previous_lfsrs, 32);
}

void load_lfsrs_into_ram(){
    read_from_flash(previous_lfsrs, 32);
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
